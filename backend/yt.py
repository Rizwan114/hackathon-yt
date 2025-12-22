import cohere
from qdrant_client import QdrantClient


cohere_client = cohere.Client("pxLHYWZhJilWoVpNolEYMcPRuoFt6C1TYclCtwjS")


qdrant = QdrantClient(
    url="https://bf55218d-3692-4c0f-93a1-d46514492ea9.us-east4-0.gcp.cloud.qdrant.io:6333", 
    api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.OdZwxQuabpvo2Eufde55QUULYP1-u117Nj53aoS1JAg",
)

def get_embadding(text):
    """Get embadding vector from cohere Embad v3"""

    response = cohere_client.embed(
        model="embed-english-v3.0",
        input_type="search_query",
        texts=[text],
    )

    return response.embeddings[0]


def retrieve(query):
    embading = get_embadding(query)
    result = qdrant.query_points(
        collection_name = "humanoid_ai_book",
        query=embading,
        limit=5
    )
    return [point.payload["text"] for point in result.points]

print(retrieve("what data do you have?"))