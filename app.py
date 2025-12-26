from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
import uuid
import time
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="Book Chatbot API",
    description="A production-ready FastAPI backend for a book chatbot",
    version="1.0.0"
)

# Add CORS middleware to allow all origins
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for development
    allow_credentials=True,
    allow_methods=["*"],  # Allow all methods
    allow_headers=["*"],  # Allow all headers
    # For production, be more specific about allowed origins
)

# Request/Response Models
class ChatRequest(BaseModel):
    message: str
    query: Optional[str] = None
    user_id: Optional[str] = None
    session_id: Optional[str] = None
    module_id: Optional[str] = None
    top_k: Optional[int] = 5
    min_similarity: Optional[float] = 0.5

class ChatResponse(BaseModel):
    response: str
    sources: List[Dict[str, Any]]

class SessionRequest(BaseModel):
    user_id: str
    initial_query: Optional[str] = None

class SessionResponse(BaseModel):
    session_id: str
    status: str
    created_at: float

class MessageRequest(BaseModel):
    message: str
    role: str  # 'user' or 'assistant'

class MessageResponse(BaseModel):
    status: str
    message_id: str

class ValidateRequest(BaseModel):
    text: str
    module_id: Optional[str] = None

class ValidateResponse(BaseModel):
    is_valid: bool
    message: str

class RetrieveRequest(BaseModel):
    query: str
    module_id: Optional[str] = None
    top_k: Optional[int] = 5
    min_similarity: Optional[float] = 0.5

class RetrieveResponse(BaseModel):
    results: List[Dict[str, Any]]
    count: int

# In-memory storage for demonstration (use database in production)
sessions: Dict[str, Dict] = {}
messages: Dict[str, List[Dict]] = {}

@app.get("/")
async def root():
    """Root endpoint to check API status"""
    return {"status": "API running"}

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "ok"}

@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Chat endpoint that processes user messages and returns responses with sources
    """
    logger.info(f"Received chat request: {request.message[:50]}...")

    # Generate mock response based on the message
    mock_responses = {
        "hello": "Hello! I'm your book chatbot assistant. How can I help you with books today?",
        "book": "I can help you find information about books! What specific book or topic are you interested in?",
        "author": "I can provide information about authors and their works. Which author would you like to know about?",
        "recommend": "I'd be happy to recommend books! Could you tell me what genre or type of book you're looking for?",
        "default": f"I received your message: '{request.message}'. As a book chatbot, I can help you find information about books, authors, and recommendations. What would you like to know?"
    }

    message_lower = request.message.lower()
    response_text = "default"

    for key in mock_responses:
        if key in message_lower:
            response_text = mock_responses[key]
            break

    # Mock sources for demonstration
    mock_sources = [
        {
            "title": "The Great Gatsby",
            "author": "F. Scott Fitzgerald",
            "content": "A classic American novel set in the summer of 1922...",
            "similarity_score": 0.85,
            "source_type": "book"
        },
        {
            "title": "To Kill a Mockingbird",
            "author": "Harper Lee",
            "content": "A gripping tale of racial injustice and childhood innocence...",
            "similarity_score": 0.78,
            "source_type": "book"
        }
    ]

    # Create session if not exists
    if request.session_id:
        session_id = request.session_id
    else:
        session_id = str(uuid.uuid4())

    if session_id not in sessions:
        sessions[session_id] = {
            "user_id": request.user_id,
            "created_at": time.time(),
            "last_activity": time.time()
        }

    # Update last activity
    sessions[session_id]["last_activity"] = time.time()

    return ChatResponse(
        response=response_text,
        sources=mock_sources
    )

@app.post("/sessions", response_model=SessionResponse)
async def create_session(request: SessionRequest):
    """
    Create a new chat session
    """
    session_id = str(uuid.uuid4())

    sessions[session_id] = {
        "user_id": request.user_id,
        "created_at": time.time(),
        "last_activity": time.time(),
        "initial_query": request.initial_query
    }

    # Initialize message history for this session
    messages[session_id] = []

    logger.info(f"Created new session: {session_id} for user: {request.user_id}")

    return SessionResponse(
        session_id=session_id,
        status="created",
        created_at=time.time()
    )

@app.post("/sessions/{session_id}/messages", response_model=MessageResponse)
async def add_message(session_id: str, request: MessageRequest):
    """
    Add a message to a specific session
    """
    if session_id not in sessions:
        raise HTTPException(status_code=404, detail="Session not found")

    message_id = str(uuid.uuid4())
    message_entry = {
        "message_id": message_id,
        "message": request.message,
        "role": request.role,
        "timestamp": time.time()
    }

    if session_id not in messages:
        messages[session_id] = []

    messages[session_id].append(message_entry)

    # Update session last activity
    sessions[session_id]["last_activity"] = time.time()

    logger.info(f"Added message to session {session_id}: {request.message[:30]}...")

    return MessageResponse(
        status="message_added",
        message_id=message_id
    )

@app.post("/validate", response_model=ValidateResponse)
async def validate_text(request: ValidateRequest):
    """
    Validate text input
    """
    logger.info(f"Validating text: {request.text[:50]}...")

    # Simple validation logic
    is_valid = len(request.text.strip()) > 0 and len(request.text) <= 1000

    if is_valid:
        message = "Text is valid"
    else:
        message = "Text is invalid - must be non-empty and less than 1000 characters"

    return ValidateResponse(
        is_valid=is_valid,
        message=message
    )

@app.post("/retrieve", response_model=RetrieveResponse)
async def retrieve_content(request: RetrieveRequest):
    """
    Retrieve relevant content based on query
    """
    logger.info(f"Retrieving content for query: {request.query[:50]}...")

    # Mock retrieval results
    mock_results = [
        {
            "id": "1",
            "title": "The Catcher in the Rye",
            "author": "J.D. Salinger",
            "content": "A story about teenage rebellion and alienation...",
            "similarity_score": 0.92,
            "source_type": "book",
            "page_numbers": [1, 2, 3]
        },
        {
            "id": "2",
            "title": "1984",
            "author": "George Orwell",
            "content": "A dystopian social science fiction novel about totalitarian control...",
            "similarity_score": 0.88,
            "source_type": "book",
            "page_numbers": [10, 15, 20]
        },
        {
            "id": "3",
            "title": "Pride and Prejudice",
            "author": "Jane Austen",
            "content": "A romantic novel of manners set in Georgian England...",
            "similarity_score": 0.81,
            "source_type": "book",
            "page_numbers": [5, 8, 12]
        }
    ]

    # Filter results based on top_k
    filtered_results = mock_results[:request.top_k]

    # Filter based on min_similarity if specified
    if request.min_similarity:
        filtered_results = [r for r in filtered_results if r["similarity_score"] >= request.min_similarity]

    return RetrieveResponse(
        results=filtered_results,
        count=len(filtered_results)
    )

# Error handlers
@app.exception_handler(404)
async def not_found_handler(request: Request, exc: HTTPException):
    return {"error": "Endpoint not found", "status_code": 404}

@app.exception_handler(500)
async def internal_error_handler(request: Request, exc: HTTPException):
    return {"error": "Internal server error", "status_code": 500}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)