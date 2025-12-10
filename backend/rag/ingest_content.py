"""
Content Ingestion Script for Physical AI Book RAG System
This script processes book content from Docusaurus markdown files,
embeds it, and stores it in the vector database for retrieval.
"""

import os
import glob
from pathlib import Path
from typing import List, Dict, Any
import markdown
from bs4 import BeautifulSoup
import asyncio
import logging
from datetime import datetime

from .embedding import get_content_embedder, ContentEmbedder
from .retrieval import get_content_retriever, VectorStore


class ContentIngestor:
    """
    Class for ingesting and processing book content for the RAG system
    """
    def __init__(self):
        self.embedder: ContentEmbedder = get_content_embedder()
        self.retriever = get_content_retriever()
        self.vector_store: VectorStore = self.retriever.vector_store
        self.logger = logging.getLogger(__name__)

    def extract_text_from_markdown(self, md_content: str) -> str:
        """
        Extract plain text from markdown content

        Args:
            md_content: Raw markdown content

        Returns:
            Plain text extracted from markdown
        """
        # Convert markdown to HTML
        html_content = markdown.markdown(md_content)

        # Parse HTML and extract text
        soup = BeautifulSoup(html_content, 'html.parser')
        text = soup.get_text(separator=' ', strip=True)

        return text

    def read_markdown_files(self, directory: str) -> List[Dict[str, Any]]:
        """
        Read all markdown files from a directory

        Args:
            directory: Directory containing markdown files

        Returns:
            List of dictionaries with file content and metadata
        """
        md_files = glob.glob(os.path.join(directory, "**/*.md"), recursive=True)
        documents = []

        for file_path in md_files:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()

                # Extract the relative path to use as section identifier
                rel_path = os.path.relpath(file_path, directory)

                # Extract title from filename or content
                title = self._extract_title(content, file_path)

                # Create document entry
                doc = {
                    'title': title,
                    'content': content,
                    'file_path': file_path,
                    'relative_path': rel_path,
                    'section': self._path_to_section(rel_path),
                    'doc_type': 'chapter',
                    'created_at': datetime.utcnow().isoformat()
                }

                documents.append(doc)
                self.logger.info(f"Read document: {title} from {rel_path}")

            except Exception as e:
                self.logger.error(f"Error reading file {file_path}: {e}")

        return documents

    def _extract_title(self, content: str, file_path: str) -> str:
        """
        Extract title from markdown content or filename

        Args:
            content: Markdown content
            file_path: File path

        Returns:
            Extracted title
        """
        # Try to extract title from frontmatter
        lines = content.split('\n')
        for i, line in enumerate(lines):
            if line.startswith('# '):  # H1 header
                title = line[2:].strip()
                return title

        # Fallback to filename
        return Path(file_path).stem.replace('-', ' ').replace('_', ' ').title()

    def _path_to_section(self, rel_path: str) -> str:
        """
        Convert relative file path to section identifier

        Args:
            rel_path: Relative path of the file

        Returns:
            Section identifier
        """
        # Remove extension and convert to a section identifier
        section = Path(rel_path).with_suffix('').as_posix()
        return section.replace('/', '-').replace('\\', '-')

    def process_documents(self, documents: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Process documents by extracting text and embedding content

        Args:
            documents: List of document dictionaries

        Returns:
            List of embedded content chunks
        """
        all_chunks = []

        for doc in documents:
            try:
                self.logger.info(f"Processing document: {doc['title']}")

                # Extract plain text from markdown
                plain_text = self.extract_text_from_markdown(doc['content'])

                # Embed the content
                embedded_chunks = self.embedder.embed_document(
                    title=doc['title'],
                    content=plain_text,
                    doc_type=doc['doc_type'],
                    section=doc['section']
                )

                # Add document metadata to each chunk
                for chunk in embedded_chunks:
                    chunk['metadata']['original_file'] = doc['file_path']
                    chunk['metadata']['relative_path'] = doc['relative_path']
                    chunk['metadata']['doc_type'] = doc['doc_type']

                all_chunks.extend(embedded_chunks)
                self.logger.info(f"Embedded {len(embedded_chunks)} chunks for {doc['title']}")

            except Exception as e:
                self.logger.error(f"Error processing document {doc['title']}: {e}")

        return all_chunks

    def ingest_from_directory(self, directory: str) -> int:
        """
        Ingest content from a directory of markdown files

        Args:
            directory: Directory containing markdown files

        Returns:
            Number of chunks ingested
        """
        self.logger.info(f"Starting content ingestion from directory: {directory}")

        # Read all markdown files
        documents = self.read_markdown_files(directory)

        if not documents:
            self.logger.warning(f"No markdown files found in directory: {directory}")
            return 0

        # Process documents and embed content
        embedded_chunks = self.process_documents(documents)

        # Index the content for retrieval
        self.retriever.index_book_content(embedded_chunks)

        self.logger.info(f"Ingested {len(embedded_chunks)} content chunks from {len(documents)} documents")
        return len(embedded_chunks)

    def ingest_single_file(self, file_path: str) -> int:
        """
        Ingest content from a single markdown file

        Args:
            file_path: Path to the markdown file

        Returns:
            Number of chunks ingested
        """
        self.logger.info(f"Starting content ingestion from file: {file_path}")

        try:
            # Read the file
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract title
            title = self._extract_title(content, file_path)

            # Extract plain text from markdown
            plain_text = self.extract_text_from_markdown(content)

            # Embed the content
            embedded_chunks = self.embedder.embed_document(
                title=title,
                content=plain_text,
                doc_type='chapter',
                section=self._path_to_section(os.path.basename(file_path))
            )

            # Index the content for retrieval
            self.retriever.index_book_content(embedded_chunks)

            self.logger.info(f"Ingested {len(embedded_chunks)} content chunks from file: {file_path}")
            return len(embedded_chunks)

        except Exception as e:
            self.logger.error(f"Error ingesting file {file_path}: {e}")
            return 0


def main():
    """
    Main function to run the content ingestion script
    """
    import argparse

    parser = argparse.ArgumentParser(description='Ingest book content for RAG system')
    parser.add_argument('--directory', type=str, help='Directory containing markdown files')
    parser.add_argument('--file', type=str, help='Single markdown file to ingest')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose logging')

    args = parser.parse_args()

    # Set up logging
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    logger = logging.getLogger(__name__)

    # Initialize content ingestor
    ingestor = ContentIngestor()

    if args.directory:
        # Ingest from directory
        if not os.path.isdir(args.directory):
            logger.error(f"Directory does not exist: {args.directory}")
            return 1

        num_chunks = ingestor.ingest_from_directory(args.directory)
        logger.info(f"Completed ingestion from directory. Total chunks: {num_chunks}")

    elif args.file:
        # Ingest single file
        if not os.path.isfile(args.file):
            logger.error(f"File does not exist: {args.file}")
            return 1

        num_chunks = ingestor.ingest_single_file(args.file)
        logger.info(f"Completed ingestion from file. Total chunks: {num_chunks}")

    else:
        logger.error("Either --directory or --file must be specified")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())