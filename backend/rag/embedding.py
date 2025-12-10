"""
Embedding Module for Physical AI Book RAG System
This module handles text embedding for the RAG system, converting book content
into vector representations for similarity search and retrieval.
"""

import os
from typing import List, Dict, Any, Optional
import numpy as np
from openai import OpenAI
import logging


class EmbeddingService:
    """
    Service class for handling text embeddings using OpenAI's embedding API
    """
    def __init__(self):
        # Initialize OpenAI client
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            raise ValueError("OPENAI_API_KEY environment variable is required")

        self.client = OpenAI(api_key=api_key)
        self.model = os.getenv('EMBEDDING_MODEL', 'text-embedding-3-small')
        self.dimension = int(os.getenv('EMBEDDING_DIMENSION', '1536'))  # Default for text-embedding-3-small

        # Set up logging
        self.logger = logging.getLogger(__name__)

        self.logger.info(f"Embedding service initialized with model: {self.model}")

    def create_embedding(self, text: str) -> List[float]:
        """
        Create embedding for a single text

        Args:
            text: Input text to embed

        Returns:
            List of embedding values
        """
        try:
            response = self.client.embeddings.create(
                input=text,
                model=self.model
            )

            embedding = response.data[0].embedding
            self.logger.debug(f"Created embedding of length {len(embedding)} for text: {text[:50]}...")

            return embedding
        except Exception as e:
            self.logger.error(f"Error creating embedding: {str(e)}")
            raise

    def create_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Create embeddings for multiple texts in batch

        Args:
            texts: List of input texts to embed

        Returns:
            List of embedding vectors
        """
        if not texts:
            return []

        try:
            # OpenAI API allows up to 2048 texts in a single request
            batch_size = min(2048, len(texts))
            all_embeddings = []

            for i in range(0, len(texts), batch_size):
                batch = texts[i:i + batch_size]

                response = self.client.embeddings.create(
                    input=batch,
                    model=self.model
                )

                batch_embeddings = [item.embedding for item in response.data]
                all_embeddings.extend(batch_embeddings)

                self.logger.debug(f"Processed batch {i//batch_size + 1}, created {len(batch_embeddings)} embeddings")

            self.logger.info(f"Created {len(all_embeddings)} embeddings for {len(texts)} texts")
            return all_embeddings
        except Exception as e:
            self.logger.error(f"Error creating embeddings: {str(e)}")
            raise

    def cosine_similarity(self, vec1: List[float], vec2: List[float]) -> float:
        """
        Calculate cosine similarity between two embedding vectors

        Args:
            vec1: First embedding vector
            vec2: Second embedding vector

        Returns:
            Cosine similarity score between -1 and 1
        """
        # Convert to numpy arrays
        v1 = np.array(vec1)
        v2 = np.array(vec2)

        # Calculate cosine similarity
        dot_product = np.dot(v1, v2)
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)

        if norm_v1 == 0 or norm_v2 == 0:
            return 0.0

        similarity = dot_product / (norm_v1 * norm_v2)
        return float(similarity)

    def get_embedding_dimensions(self) -> int:
        """
        Get the expected dimension of embeddings

        Returns:
            Embedding dimension
        """
        return self.dimension


class ContentEmbedder:
    """
    High-level service for embedding book content with chunking and metadata
    """
    def __init__(self):
        self.embedding_service = EmbeddingService()
        self.logger = logging.getLogger(__name__)

    def chunk_text(self, text: str, chunk_size: int = 512, overlap: int = 50) -> List[Dict[str, Any]]:
        """
        Split text into overlapping chunks for better retrieval

        Args:
            text: Input text to chunk
            chunk_size: Maximum size of each chunk (in tokens/words)
            overlap: Number of tokens/words to overlap between chunks

        Returns:
            List of chunk dictionaries with text and metadata
        """
        # Simple word-based chunking (in production, you might want to use token-based chunking)
        words = text.split()
        chunks = []

        start_idx = 0
        while start_idx < len(words):
            end_idx = min(start_idx + chunk_size, len(words))
            chunk_words = words[start_idx:end_idx]
            chunk_text = ' '.join(chunk_words)

            chunk = {
                'text': chunk_text,
                'start_idx': start_idx,
                'end_idx': end_idx,
                'chunk_id': f"chunk_{len(chunks)}",
                'length': len(chunk_words)
            }

            chunks.append(chunk)

            # Move to next chunk with overlap
            start_idx = end_idx - overlap
            if start_idx < end_idx:  # Ensure we make progress
                start_idx = end_idx

        self.logger.info(f"Chunked text into {len(chunks)} chunks")
        return chunks

    def embed_content(self, content: str, metadata: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
        """
        Embed content with metadata and chunking

        Args:
            content: Book content to embed
            metadata: Additional metadata about the content

        Returns:
            List of embedded chunks with metadata
        """
        if not content.strip():
            return []

        # Chunk the content
        chunks = self.chunk_text(content)

        # Extract text from chunks for embedding
        texts = [chunk['text'] for chunk in chunks]

        # Create embeddings
        embeddings = self.embedding_service.create_embeddings(texts)

        # Combine chunks with embeddings and metadata
        embedded_chunks = []
        for chunk, embedding in zip(chunks, embeddings):
            embedded_chunk = {
                'id': chunk['chunk_id'],
                'text': chunk['text'],
                'embedding': embedding,
                'metadata': {
                    **(metadata or {}),
                    'start_idx': chunk['start_idx'],
                    'end_idx': chunk['end_idx'],
                    'length': chunk['length']
                }
            }
            embedded_chunks.append(embedded_chunk)

        self.logger.info(f"Embedded {len(embedded_chunks)} content chunks")
        return embedded_chunks

    def embed_document(self,
                     title: str,
                     content: str,
                     doc_type: str = "chapter",
                     section: str = "",
                     page_numbers: Optional[List[int]] = None) -> List[Dict[str, Any]]:
        """
        Embed a complete document with comprehensive metadata

        Args:
            title: Document title
            content: Document content
            doc_type: Type of document (chapter, section, etc.)
            section: Section identifier
            page_numbers: List of page numbers for the content

        Returns:
            List of embedded chunks with document metadata
        """
        metadata = {
            'title': title,
            'doc_type': doc_type,
            'section': section,
            'page_numbers': page_numbers or [],
            'content_length': len(content),
            'created_at': self._get_timestamp()
        }

        return self.embed_content(content, metadata)

    def _get_timestamp(self) -> str:
        """
        Get current timestamp as string

        Returns:
            ISO format timestamp string
        """
        from datetime import datetime
        return datetime.utcnow().isoformat()


# Global instance for easy access
embedding_service = EmbeddingService()
content_embedder = ContentEmbedder()


def get_embedding_service() -> EmbeddingService:
    """
    Get the global embedding service instance

    Returns:
        EmbeddingService instance
    """
    return embedding_service


def get_content_embedder() -> ContentEmbedder:
    """
    Get the global content embedder instance

    Returns:
        ContentEmbedder instance
    """
    return content_embedder


# Example usage
if __name__ == "__main__":
    # Example of how to use the embedding service
    embedder = get_content_embedder()

    sample_content = '''
    Chapter 1: Introduction to the Robotic Nervous System
    The robotic nervous system represents a paradigm shift in how we approach
    humanoid robotics. By treating the robot's communication infrastructure as
    its nervous system, we can better understand the complex interactions between
    perception, planning, and actuation layers.

    This system follows a hierarchical architecture where low-level sensorimotor
    functions are handled by fast, reactive controllers, while high-level cognitive
    functions operate at slower timescales but with greater complexity and
    adaptability.
    '''

    embedded_chunks = embedder.embed_document(
        title="Chapter 1: Introduction to the Robotic Nervous System",
        content=sample_content,
        doc_type="chapter",
        section="module-1-ros2"
    )

    print(f"Embedded {len(embedded_chunks)} chunks")
    print(f"First chunk embedding length: {len(embedded_chunks[0]['embedding'])}")
    print(f"First chunk text preview: {embedded_chunks[0]['text'][:100]}...")