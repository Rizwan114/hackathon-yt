"""
FastAPI application for the Physical AI Book RAG system
"""
from fastapi import FastAPI, HTTPException, BackgroundTasks
from pydantic import BaseModel
from typing import List, Dict, Optional
import logging
import os
from contextlib import asynccontextmanager
from datetime import datetime

from database.connection import db
from rag.chat import get_conversation_manager, ChatSession, MessageRole
from rag.retrieval import get_content_retriever
from rag.embedding import get_content_embedder

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize the conversation manager
conversation_manager = get_conversation_manager()

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    FastAPI lifespan event handler
    """
    # Startup
    logger.info("Starting up the Physical AI Book RAG system")
    await db.connect()
    yield
    # Shutdown
    logger.info("Shutting down the Physical AI Book RAG system")
    await db.disconnect()

# Create FastAPI app
app = FastAPI(
    title="Physical AI Book RAG API",
    description="API for the Physical AI Book with RAG capabilities",
    version="0.1.0",
    lifespan=lifespan
)

# Request/Response models
class ChatRequest(BaseModel):
    message: str
    user_id: str
    session_id: Optional[str] = None
    module_id: Optional[str] = "module-1-ros2"  # Default module
    top_k: int = 5
    min_similarity: float = 0.3

class SourceReference(BaseModel):
    module: str
    chapter: str
    section: str
    content: str
    similarity_score: float

class ChatResponse(BaseModel):
    response: str
    sources: List[SourceReference]
    session_id: str
    timestamp: str

class ValidationRequest(BaseModel):
    text: str
    module_id: Optional[str] = "module-1-ros2"

class ValidationResponse(BaseModel):
    valid: bool
    matches: List[Dict]

class RetrieveRequest(BaseModel):
    query: str
    module_id: Optional[str] = "module-1-ros2"
    top_k: int = 5
    min_similarity: float = 0.3

class RetrieveResponse(BaseModel):
    results: List[Dict]

class SessionCreateRequest(BaseModel):
    user_id: str
    initial_query: Optional[str] = None

class SessionResponse(BaseModel):
    session_id: str
    user_id: str
    created_at: str
    message_count: int

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Chat endpoint for interacting with the Physical AI Book content using RAG
    """
    try:
        # Get or create session
        if request.session_id:
            # In a real implementation, you'd retrieve the existing session from storage
            # For now, we'll create a temporary session for this interaction
            session = conversation_manager.chat_service.create_session(request.user_id)
        else:
            session = conversation_manager.chat_service.create_session(request.user_id)

        # Add user message to session
        conversation_manager.chat_service.add_message(
            session, MessageRole.USER, request.message
        )

        # Generate response using RAG
        response_text = conversation_manager.chat_service.generate_response(
            session, request.message
        )

        # Retrieve sources for the response
        retrieved_chunks = conversation_manager.chat_service.get_relevant_sources(
            session, request.message, top_k=request.top_k
        )

        # Format sources
        sources = []
        for chunk in retrieved_chunks:
            source = SourceReference(
                module=request.module_id,
                chapter=chunk.source_title,
                section=chunk.source_section,
                content=chunk.text[:500] + "..." if len(chunk.text) > 500 else chunk.text,
                similarity_score=chunk.similarity_score
            )
            sources.append(source)

        # Create response
        response = ChatResponse(
            response=response_text,
            sources=sources,
            session_id=session.id,
            timestamp=datetime.utcnow().isoformat()
        )

        return response
    except Exception as e:
        logger.error(f"Error in chat endpoint: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/sessions", response_model=SessionResponse)
async def create_session_endpoint(request: SessionCreateRequest):
    """
    Create a new chat session
    """
    try:
        session = conversation_manager.start_conversation(
            request.user_id,
            initial_query=request.initial_query
        )

        response = SessionResponse(
            session_id=session.id,
            user_id=session.user_id,
            created_at=session.created_at.isoformat(),
            message_count=len(session.messages)
        )

        return response
    except Exception as e:
        logger.error(f"Error in create session endpoint: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/sessions/{session_id}/messages", response_model=ChatResponse)
async def add_message_endpoint(session_id: str, request: ChatRequest):
    """
    Add a message to an existing session and get a response
    """
    try:
        # In a real implementation, you'd retrieve the session from storage
        # For now, we'll create a temporary session
        session = conversation_manager.chat_service.create_session(request.user_id)
        session.id = session_id  # Set the provided session ID

        # Add user message to session
        conversation_manager.chat_service.add_message(
            session, MessageRole.USER, request.message
        )

        # Generate response using RAG
        response_text = conversation_manager.chat_service.generate_response(
            session, request.message
        )

        # Retrieve sources for the response
        retrieved_chunks = conversation_manager.chat_service.get_relevant_sources(
            session, request.message, top_k=request.top_k
        )

        # Format sources
        sources = []
        for chunk in retrieved_chunks:
            source = SourceReference(
                module=request.module_id,
                chapter=chunk.source_title,
                section=chunk.source_section,
                content=chunk.text[:500] + "..." if len(chunk.text) > 500 else chunk.text,
                similarity_score=chunk.similarity_score
            )
            sources.append(source)

        # Add assistant response to session
        conversation_manager.chat_service.add_message(
            session, MessageRole.ASSISTANT, response_text
        )

        response = ChatResponse(
            response=response_text,
            sources=sources,
            session_id=session.id,
            timestamp=datetime.utcnow().isoformat()
        )

        return response
    except Exception as e:
        logger.error(f"Error in add message endpoint: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/validate", response_model=ValidationResponse)
async def validate_endpoint(request: ValidationRequest):
    """
    Validate user-selected text against book content for grounding verification
    """
    try:
        # Get the content retriever
        retriever = get_content_retriever()

        # Search for similar content to the provided text
        retrieved_chunks = retriever.search_book_content(
            query=request.text,
            top_k=3,
            min_similarity=0.5
        )

        # Format matches
        matches = []
        for chunk in retrieved_chunks:
            match = {
                "source": f"{chunk.source_title} ({chunk.source_section})",
                "similarity": chunk.similarity_score,
                "content": chunk.text[:200] + "..." if len(chunk.text) > 200 else chunk.text,
            }
            matches.append(match)

        result = ValidationResponse(
            valid=len(matches) > 0,
            matches=matches
        )

        return result
    except Exception as e:
        logger.error(f"Error in validate endpoint: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/retrieve", response_model=RetrieveResponse)
async def retrieve_endpoint(request: RetrieveRequest):
    """
    Retrieve relevant book content based on query for RAG system
    """
    try:
        # Get the content retriever
        retriever = get_content_retriever()

        # Search for relevant content
        retrieved_chunks = retriever.search_book_content(
            query=request.query,
            section_filter=None,  # Could filter by section if needed
            top_k=request.top_k,
            min_similarity=request.min_similarity
        )

        # Format results
        results = []
        for chunk in retrieved_chunks:
            result = {
                "module": request.module_id,
                "chapter": chunk.source_title,
                "section": chunk.source_section,
                "content": chunk.text,
                "similarity_score": chunk.similarity_score,
                "metadata": chunk.metadata
            }
            results.append(result)

        response = RetrieveResponse(results=results)
        return response
    except Exception as e:
        logger.error(f"Error in retrieve endpoint: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy", "service": "Physical AI Book RAG API", "timestamp": datetime.utcnow().isoformat()}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)


# The book ingestion code is kept separately below
import requests
import xml.etree.ElementTree as ET
import trafilatura
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import cohere

# -------------------------------------
# CONFIG
# -------------------------------------
# Your Deployment Link:
SITEMAP_URL = "https://hackathon-yt-y97j.vercel.app/sitemap.xml"
COLLECTION_NAME = "humanoid_ai_book"

cohere_client = cohere.Client("Weyiywa7HxvabgJtc96onPL7rLTvQGsdmvLzYaCC")
EMBED_MODEL = "embed-english-v3.0"

# Connect to Qdrant Cloud
qdrant = QdrantClient(
    url="https://bf55218d-3692-4c0f-93a1-d46514492ea9.us-east4-0.gcp.cloud.qdrant.io:6333",
    api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.OdZwxQuabpvo2Eufde55QUULYP1-u117Nj53aoS1JAg",
)

# -------------------------------------
# Step 1 — Extract URLs from sitemap
# -------------------------------------
def get_all_urls(sitemap_url):
    xml = requests.get(sitemap_url).text
    root = ET.fromstring(xml)

    urls = []
    for child in root:
        loc_tag = child.find("{http://www.sitemaps.org/schemas/sitemap/0.9}loc")
        if loc_tag is not None:
            urls.append(loc_tag.text)

    print("\nFOUND URLS:")
    for u in urls:
        print(" -", u)

    return urls


# -------------------------------------
# Step 2 — Download page + extract text
# -------------------------------------
def extract_text_from_url(url):
    html = requests.get(url).text
    text = trafilatura.extract(html)

    if not text:
        print("[WARNING] No text extracted from:", url)

    return text


# -------------------------------------
# Step 3 — Chunk the text
# -------------------------------------
def chunk_text(text, max_chars=1200):
    chunks = []
    while len(text) > max_chars:
        split_pos = text[:max_chars].rfind(". ")
        if split_pos == -1:
            split_pos = max_chars
        chunks.append(text[:split_pos])
        text = text[split_pos:]
    chunks.append(text)
    return chunks


# -------------------------------------
# Step 4 — Create embedding
# -------------------------------------
def embed(text):
    response = cohere_client.embed(
        model=EMBED_MODEL,
        input_type="search_query",  # Use search_query for queries
        texts=[text],
    )
    return response.embeddings[0]  # Return the first embedding


# -------------------------------------
# Step 5 — Store in Qdrant
# -------------------------------------
def create_collection():
    print("\nCreating Qdrant collection...")
    qdrant.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(
        size=1024,        # Cohere embed-english-v3.0 dimension
        distance=Distance.COSINE
        )
    )

def save_chunk_to_qdrant(chunk, chunk_id, url):
    vector = embed(chunk)

    qdrant.upsert(
        collection_name=COLLECTION_NAME,
        points=[
            PointStruct(
                id=chunk_id,
                vector=vector,
                payload={
                    "url": url,
                    "text": chunk,
                    "chunk_id": chunk_id
                }
            )
        ]
    )


# -------------------------------------
# MAIN INGESTION PIPELINE
# -------------------------------------
def ingest_book():
    urls = get_all_urls(SITEMAP_URL)

    create_collection()

    global_id = 1

    for url in urls:
        print("\nProcessing:", url)
        text = extract_text_from_url(url)

        if not text:
            continue

        chunks = chunk_text(text)

        for ch in chunks:
            save_chunk_to_qdrant(ch, global_id, url)
            print(f"Saved chunk {global_id}")
            global_id += 1

    print("\n✔️ Ingestion completed!")
    print("Total chunks stored:", global_id - 1)


if __name__ == "__main__":
    ingest_book()