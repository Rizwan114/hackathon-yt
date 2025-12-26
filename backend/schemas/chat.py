"""
Chat-related data models for API requests and responses
"""
from pydantic import BaseModel
from typing import List, Dict, Optional
from datetime import datetime

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