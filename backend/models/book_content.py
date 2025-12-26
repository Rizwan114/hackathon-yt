"""
Data model for Book Content entity
"""
from pydantic import BaseModel
from typing import Optional


class BookContent(BaseModel):
    """
    Represents the source material that the chatbot uses to answer questions
    """
    id: Optional[int] = None
    text: str  # the actual content
    url: str   # source URL
    chunk_id: int  # identifier for the chunk
    embedding: Optional[list] = None  # Cohere embedding of the text (vector)

    class Config:
        # Validation: Text must not be empty
        @classmethod
        def validate_text(cls, v):
            if not v or not v.strip():
                raise ValueError('Text must not be empty')
            return v