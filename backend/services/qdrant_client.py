"""
Service wrapper for Qdrant vector database client
"""
import logging
from typing import List, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
import os

logger = logging.getLogger(__name__)


class QdrantService:
    """
    Service wrapper for Qdrant vector database operations
    """

    def __init__(self):
        # Initialize Qdrant client with environment variables
        self.client = QdrantClient(
            url=os.getenv("QDRANT_URL", "https://bf55218d-3692-4c0f-93a1-d46514492ea9.us-east4-0.gcp.cloud.qdrant.io:6333"),
            api_key=os.getenv("QDRANT_API_KEY", "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.OdZwxQuabpvo2Eufde55QUULYP1-u117Nj53aoS1JAg"),
        )
        self.collection_name = "humanoid_ai_book"

    def search(self, query_vector: List[float], top_k: int = 5) -> List[dict]:
        """
        Search for similar vectors in the Qdrant collection
        """
        try:
            # Perform vector similarity search
            search_result = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k
            )

            # Format results
            results = []
            for point in search_result:
                results.append({
                    "id": point.id,
                    "payload": point.payload,
                    "score": point.score
                })

            return results
        except Exception as e:
            logger.error(f"Error searching in Qdrant: {e}")
            return []

    def get_point(self, point_id: int) -> Optional[dict]:
        """
        Get a specific point from the collection
        """
        try:
            points = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[point_id]
            )

            if points:
                point = points[0]
                return {
                    "id": point.id,
                    "payload": point.payload
                }

            return None
        except Exception as e:
            logger.error(f"Error getting point from Qdrant: {e}")
            return None

    def health_check(self) -> bool:
        """
        Check if Qdrant is accessible
        """
        try:
            # Try to get collection info to verify connection
            collection_info = self.client.get_collection(self.collection_name)
            return True
        except Exception as e:
            logger.error(f"Qdrant health check failed: {e}")
            return False