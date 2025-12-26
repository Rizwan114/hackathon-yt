"""
Service for handling text embeddings using Cohere
"""
import logging
import os
from typing import List, Union
import cohere

logger = logging.getLogger(__name__)


class EmbeddingService:
    """
    Service for generating text embeddings using Cohere
    """

    def __init__(self):
        # Initialize Cohere client
        self.client = cohere.Client(
            os.getenv("COHERE_API_KEY", "Weyiywa7HxvabgJtc96onPL7rLTvQGsdmvLzYaCC")
        )
        self.model = "embed-english-v3.0"

    def embed_text(self, text: str, input_type: str = "search_query") -> List[float]:
        """
        Generate embedding for a single text
        """
        try:
            response = self.client.embed(
                model=self.model,
                input_type=input_type,  # Use search_query for queries
                texts=[text],
            )
            return response.embeddings[0]  # Return the first embedding
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            raise

    def embed_texts(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generate embeddings for multiple texts
        """
        try:
            response = self.client.embed(
                model=self.model,
                input_type=input_type,
                texts=texts,
            )
            return [embedding for embedding in response.embeddings]
        except Exception as e:
            logger.error(f"Error generating embeddings: {e}")
            raise

    def get_embedding_dimensions(self) -> int:
        """
        Get the expected dimensions for embeddings from this model
        """
        # Cohere embed-english-v3.0 returns 1024-dimensional vectors
        return 1024