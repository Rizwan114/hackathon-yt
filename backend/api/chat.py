"""
API router for chat endpoints
"""
from fastapi import APIRouter, HTTPException
from typing import List
from datetime import datetime
import logging

from agents import Agent, Runner, OpenAIChatCompletionsModel, AsyncOpenAI
import os
from dotenv import load_dotenv
from agents import function_tool

from schemas.chat import ChatRequest, ChatResponse, SourceReference
from services.retrieval_service import RetrievalService
from services.qdrant_client import QdrantService
from services.embedding_service import EmbeddingService

# Load environment variables
load_dotenv()

# Set up logging
logger = logging.getLogger(__name__)

# Initialize API router
router = APIRouter()

# Initialize services
retrieval_service = RetrievalService()
qdrant_service = QdrantService()
embedding_service = EmbeddingService()

# Initialize agents components
gemini_api_key = os.getenv("GEMINI_API_KEY")
if not gemini_api_key:
    logger.error("GEMINI_API_KEY not found in environment variables")
    raise ValueError("GEMINI_API_KEY not found in environment variables")

provider = AsyncOpenAI(
    api_key=gemini_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

model = OpenAIChatCompletionsModel(
    model="gemini-2.0-flash",
    openai_client=provider
)

# Create a function tool that uses our services
@function_tool
def retrieve(query: str) -> List[str]:
    try:
        # Use the retrieval service to get relevant content
        sources = retrieval_service.retrieve_content(query, top_k=5)
        # Extract just the content from the sources
        content_list = [source.content for source in sources]
        return content_list
    except Exception as e:
        logger.error(f"Error in retrieve function: {e}")
        return ["Error retrieving information from the book. Please try again later."]

# Create the agent
agent = Agent(
    name="Assistant",
    instructions="""
You are an AI tutor for the Physical AI & Humanoid Robotics textbook.
To answer the user question, first call the tool `retrieve` with the user query.
Use ONLY the returned content from `retrieve` to answer.
If the answer is not in the retrieved content, say "I don't know".
""",
    model=model,
    tools=[retrieve]
)

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Chat endpoint for interacting with the Physical AI Book content using RAG
    """
    try:
        # Get relevant sources using the retrieval service first
        sources = retrieval_service.retrieve_content(request.message, request.top_k, request.min_similarity)

        # If no relevant sources found, inform the agent
        if not sources or all(source.similarity_score < request.min_similarity for source in sources):
            # Run the agent with a specific instruction that no relevant content was found
            result = Runner.run_sync(
                agent,
                input=f"{request.message} (Note: No relevant content found in the book. Respond accordingly.)",
            )
        else:
            # Run the agent with the user's query
            result = Runner.run_sync(
                agent,
                input=request.message,
            )

        # Validate that the response is based on book content
        # In a real implementation, this would check if the response is grounded in the sources
        response_text = result.final_output

        # Create response
        response = ChatResponse(
            response=response_text,
            sources=sources,
            session_id=request.session_id or "session_" + str(hash(request.user_id + request.message))[:8],
            timestamp=datetime.utcnow().isoformat()
        )

        return response
    except Exception as e:
        logger.error(f"Error in chat endpoint: {e}")
        raise HTTPException(status_code=500, detail=str(e))