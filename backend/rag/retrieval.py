"""
Retrieval Module for Physical AI Book RAG System
This module handles content retrieval using vector similarity search
to find relevant book content for user queries.
"""

import os
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
import numpy as np
from .embedding import get_embedding_service, EmbeddingService
import logging
import sqlite3
import json
from datetime import datetime


@dataclass
class RetrievedChunk:
    """
    Data class representing a retrieved content chunk
    """
    id: str
    text: str
    metadata: Dict[str, Any]
    similarity_score: float
    source_title: str
    source_section: str


class VectorStore:
    """
    Simple in-memory vector store for demonstration purposes.
    In production, this would interface with a proper vector database like Qdrant, Pinecone, etc.
    """
    def __init__(self):
        self.chunks: List[Dict[str, Any]] = []
        self.logger = logging.getLogger(__name__)

    def add_chunks(self, chunks: List[Dict[str, Any]]):
        """
        Add embedded chunks to the vector store

        Args:
            chunks: List of embedded chunks to add
        """
        self.chunks.extend(chunks)
        self.logger.info(f"Added {len(chunks)} chunks to vector store. Total: {len(self.chunks)}")

    def search(self, query_embedding: List[float], top_k: int = 5) -> List[RetrievedChunk]:
        """
        Search for similar chunks to the query embedding

        Args:
            query_embedding: Embedding vector for the query
            top_k: Number of top results to return

        Returns:
            List of retrieved chunks sorted by similarity
        """
        if not self.chunks:
            return []

        # Calculate similarity scores for all chunks
        similarities = []
        for chunk in self.chunks:
            similarity = self._cosine_similarity(query_embedding, chunk['embedding'])
            similarities.append((chunk, similarity))

        # Sort by similarity (descending)
        similarities.sort(key=lambda x: x[1], reverse=True)

        # Return top_k results
        results = []
        for chunk, similarity in similarities[:top_k]:
            retrieved_chunk = RetrievedChunk(
                id=chunk['id'],
                text=chunk['text'],
                metadata=chunk['metadata'],
                similarity_score=similarity,
                source_title=chunk['metadata'].get('title', 'Unknown'),
                source_section=chunk['metadata'].get('section', 'Unknown')
            )
            results.append(retrieved_chunk)

        self.logger.debug(f"Search returned {len(results)} results")
        return results

    def _cosine_similarity(self, vec1: List[float], vec2: List[float]) -> float:
        """
        Calculate cosine similarity between two vectors

        Args:
            vec1: First vector
            vec2: Second vector

        Returns:
            Cosine similarity score
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

    def get_chunk_by_id(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific chunk by its ID

        Args:
            chunk_id: ID of the chunk to retrieve

        Returns:
            Chunk data if found, None otherwise
        """
        for chunk in self.chunks:
            if chunk['id'] == chunk_id:
                return chunk
        return None


class ContentRetriever:
    """
    High-level service for content retrieval with query processing and result ranking
    """
    def __init__(self):
        self.embedding_service: EmbeddingService = get_embedding_service()
        self.vector_store = VectorStore()
        self.logger = logging.getLogger(__name__)

    def index_content(self, content_chunks: List[Dict[str, Any]]):
        """
        Index content chunks for retrieval

        Args:
            content_chunks: List of embedded content chunks to index
        """
        self.vector_store.add_chunks(content_chunks)
        self.logger.info(f"Indexed {len(content_chunks)} content chunks for retrieval")

    def retrieve_content(self, query: str, top_k: int = 5, min_similarity: float = 0.3) -> List[RetrievedChunk]:
        """
        Retrieve content relevant to the query

        Args:
            query: User query to search for
            top_k: Maximum number of results to return
            min_similarity: Minimum similarity threshold

        Returns:
            List of relevant content chunks
        """
        try:
            # Create embedding for the query
            query_embedding = self.embedding_service.create_embedding(query)
            self.logger.debug(f"Created query embedding of length {len(query_embedding)}")

            # Search in the vector store
            results = self.vector_store.search(query_embedding, top_k=top_k)

            # Filter by minimum similarity
            filtered_results = [result for result in results if result.similarity_score >= min_similarity]

            self.logger.info(f"Retrieved {len(filtered_results)} relevant chunks for query: {query[:50]}...")
            return filtered_results
        except Exception as e:
            self.logger.error(f"Error retrieving content: {str(e)}")
            raise

    def retrieve_content_with_context(self, query: str, top_k: int = 3, context_window: int = 1) -> List[RetrievedChunk]:
        """
        Retrieve content with surrounding context chunks

        Args:
            query: User query to search for
            top_k: Number of main results to return
            context_window: Number of neighboring chunks to include as context

        Returns:
            List of content chunks with context
        """
        # First, retrieve the main results
        main_results = self.retrieve_content(query, top_k=top_k * 2)  # Get more to account for context

        # For each result, try to include context chunks
        enhanced_results = []
        for result in main_results:
            # Try to get context from metadata
            if 'start_idx' in result.metadata and 'end_idx' in result.metadata:
                # In a real implementation, you'd retrieve neighboring chunks
                # For now, we'll just return the main result
                enhanced_results.append(result)
            else:
                enhanced_results.append(result)

        return enhanced_results[:top_k]

    def search_by_section(self, query: str, section: str, top_k: int = 5) -> List[RetrievedChunk]:
        """
        Search for content within a specific section

        Args:
            query: User query to search for
            section: Section to search within
            top_k: Maximum number of results to return

        Returns:
            List of relevant content chunks from the specified section
        """
        # Create embedding for the query
        query_embedding = self.embedding_service.create_embedding(query)

        # Filter chunks by section first
        section_chunks = [
            chunk for chunk in self.vector_store.chunks
            if chunk['metadata'].get('section', '') == section
        ]

        # Calculate similarities for section chunks only
        similarities = []
        for chunk in section_chunks:
            similarity = self.vector_store._cosine_similarity(query_embedding, chunk['embedding'])
            similarities.append((chunk, similarity))

        # Sort by similarity and return top results
        similarities.sort(key=lambda x: x[1], reverse=True)
        results = []
        for chunk, similarity in similarities[:top_k]:
            retrieved_chunk = RetrievedChunk(
                id=chunk['id'],
                text=chunk['text'],
                metadata=chunk['metadata'],
                similarity_score=similarity,
                source_title=chunk['metadata'].get('title', 'Unknown'),
                source_section=chunk['metadata'].get('section', 'Unknown')
            )
            results.append(retrieved_chunk)

        self.logger.info(f"Found {len(results)} results in section '{section}' for query: {query[:50]}...")
        return results

    def get_content_by_id(self, chunk_id: str) -> Optional[RetrievedChunk]:
        """
        Retrieve a specific chunk by its ID

        Args:
            chunk_id: ID of the chunk to retrieve

        Returns:
            Retrieved chunk if found, None otherwise
        """
        chunk = self.vector_store.get_chunk_by_id(chunk_id)
        if chunk:
            return RetrievedChunk(
                id=chunk['id'],
                text=chunk['text'],
                metadata=chunk['metadata'],
                similarity_score=1.0,  # Exact match
                source_title=chunk['metadata'].get('title', 'Unknown'),
                source_section=chunk['metadata'].get('section', 'Unknown')
            )
        return None

    def get_similar_chunks(self, chunk_id: str, top_k: int = 5) -> List[RetrievedChunk]:
        """
        Find chunks similar to a given chunk

        Args:
            chunk_id: ID of the reference chunk
            top_k: Number of similar chunks to return

        Returns:
            List of similar content chunks
        """
        reference_chunk = self.vector_store.get_chunk_by_id(chunk_id)
        if not reference_chunk:
            return []

        reference_embedding = reference_chunk['embedding']

        # Calculate similarities with all other chunks
        similarities = []
        for chunk in self.vector_store.chunks:
            if chunk['id'] != chunk_id:  # Don't include the reference chunk itself
                similarity = self.vector_store._cosine_similarity(reference_embedding, chunk['embedding'])
                similarities.append((chunk, similarity))

        # Sort and return top_k
        similarities.sort(key=lambda x: x[1], reverse=True)
        results = []
        for chunk, similarity in similarities[:top_k]:
            retrieved_chunk = RetrievedChunk(
                id=chunk['id'],
                text=chunk['text'],
                metadata=chunk['metadata'],
                similarity_score=similarity,
                source_title=chunk['metadata'].get('title', 'Unknown'),
                source_section=chunk['metadata'].get('section', 'Unknown')
            )
            results.append(retrieved_chunk)

        return results


class BookContentRetriever:
    """
    Specialized retriever for book content with additional features
    like section filtering, chapter navigation, and content hierarchy
    """
    def __init__(self):
        self.content_retriever = ContentRetriever()
        self.logger = logging.getLogger(__name__)

    def search_book_content(self, query: str,
                           section_filter: Optional[str] = None,
                           top_k: int = 5,
                           min_similarity: float = 0.3) -> List[RetrievedChunk]:
        """
        Search book content with optional section filtering

        Args:
            query: User query to search for
            section_filter: Optional section to filter results by
            top_k: Maximum number of results to return
            min_similarity: Minimum similarity threshold

        Returns:
            List of relevant book content chunks
        """
        if section_filter:
            return self.content_retriever.search_by_section(
                query, section_filter, top_k
            )
        else:
            return self.content_retriever.retrieve_content(
                query, top_k, min_similarity
            )

    def find_related_content(self, query: str, context_chunk_id: str, top_k: int = 5) -> List[RetrievedChunk]:
        """
        Find content related to both the query and a specific context chunk

        Args:
            query: User query
            context_chunk_id: ID of context chunk to provide focus
            top_k: Number of results to return

        Returns:
            List of related content chunks
        """
        # Get the context chunk
        context_chunk = self.content_retriever.get_content_by_id(context_chunk_id)
        if not context_chunk:
            return self.content_retriever.retrieve_content(query, top_k)

        # Create a combined query embedding that considers both the query and context
        query_embedding = self.content_retriever.embedding_service.create_embedding(query)
        context_embedding = self.content_retriever.embedding_service.create_embedding(context_chunk.text)

        # Combine embeddings (simple average for now)
        combined_embedding = [
            (q + c) / 2.0 for q, c in zip(query_embedding, context_embedding)
        ]

        # Search using the combined embedding
        results = self._search_with_embedding(combined_embedding, top_k)
        return results

    def _search_with_embedding(self, embedding: List[float], top_k: int) -> List[RetrievedChunk]:
        """
        Search using a pre-computed embedding

        Args:
            embedding: Pre-computed embedding vector
            top_k: Number of results to return

        Returns:
            List of retrieved chunks
        """
        results = self.content_retriever.vector_store.search(embedding, top_k)
        return results

    def index_book_content(self, book_content: List[Dict[str, Any]]):
        """
        Index book content for retrieval

        Args:
            book_content: List of embedded book content chunks
        """
        self.content_retriever.index_content(book_content)
        self.logger.info(f"Indexed {len(book_content)} book content chunks")


# Global instance for easy access
content_retriever = BookContentRetriever()


def get_content_retriever() -> BookContentRetriever:
    """
    Get the global content retriever instance

    Returns:
        BookContentRetriever instance
    """
    return content_retriever


# Example usage
if __name__ == "__main__":
    # Example of how to use the retrieval service
    retriever = get_content_retriever()

    # This would typically be called after content is indexed
    # For example:
    # results = retriever.search_book_content("What is the robotic nervous system?", top_k=3)
    # print(f"Found {len(results)} relevant chunks")
    # for result in results:
    #     print(f"Score: {result.similarity_score:.3f}, Title: {result.source_title}")
    #     print(f"Content: {result.text[:100]}...")
    #     print("---")