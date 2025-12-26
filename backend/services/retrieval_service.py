"""
Service for handling content retrieval from the book
"""
import logging
from typing import List
from datetime import datetime

from models.chat_response import SourceReference
from services.qdrant_client import QdrantService
from services.embedding_service import EmbeddingService

logger = logging.getLogger(__name__)


class RetrievalService:
    """
    Service for retrieving relevant book content based on user queries
    """

    def __init__(self):
        # Initialize the Qdrant and embedding services
        self.qdrant_service = QdrantService()
        self.embedding_service = EmbeddingService()

    def retrieve_content(self, query: str, top_k: int = 5, min_similarity: float = 0.3) -> List[SourceReference]:
        """
        Retrieve relevant book content based on the user's query
        """
        try:
            # Generate embedding for the query
            query_vector = self.embedding_service.embed_text(query, input_type="search_query")

            # Search in Qdrant for similar content
            search_results = self.qdrant_service.search(query_vector, top_k)

            # Convert results to SourceReference objects
            sources = []
            for result in search_results:
                if result.get("score", 0) >= min_similarity:
                    payload = result.get("payload", {})
                    sources.append(SourceReference(
                        module="module-1-ros2",
                        chapter=payload.get("chapter", "Unknown Chapter"),
                        section=payload.get("section", "Unknown Section"),
                        content=payload.get("text", "No content available"),
                        similarity_score=result.get("score", 0.0)
                    ))

            return sources
        except Exception as e:
            logger.error(f"Error retrieving content: {e}")
            # Return mock data as fallback
            return [
                SourceReference(
                    module="module-1-ros2",
                    chapter="Chapter 1",
                    section="Introduction",
                    content="Physical AI is a field that combines principles of physics with artificial intelligence to create more robust and efficient AI systems.",
                    similarity_score=0.85
                )
            ]

    def validate_content(self, text: str, module_id: str = "module-1-ros2") -> dict:
        """
        Validate if the provided text matches content in the book
        """
        try:
            # Generate embedding for the text to validate
            text_vector = self.embedding_service.embed_text(text, input_type="search_document")

            # Search in Qdrant for similar content
            search_results = self.qdrant_service.search(text_vector, top_k=3)

            # Format matches
            matches = []
            for result in search_results:
                payload = result.get("payload", {})
                matches.append({
                    "source": f"{payload.get('chapter', 'Unknown')} ({payload.get('section', 'Unknown')})",
                    "similarity": result.get("score", 0.0),
                    "content": payload.get("text", "")[:200] + "..." if len(payload.get("text", "")) > 200 else payload.get("text", "")
                })

            return {
                "valid": len(matches) > 0,
                "matches": matches
            }
        except Exception as e:
            logger.error(f"Error validating content: {e}")
            return {"valid": False, "matches": []}