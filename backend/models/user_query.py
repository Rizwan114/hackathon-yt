"""
Data model for User Query entity
"""
from pydantic import BaseModel
from typing import Optional
from datetime import datetime


class UserQuery(BaseModel):
    """
    Represents a question or request submitted by the user
    """
    query_text: str  # the user's question
    user_id: str     # identifier for the user
    timestamp: Optional[datetime] = None  # when the query was made

    class Config:
        # Validation: Query text must not be empty
        @classmethod
        def validate_query_text(cls, v):
            if not v or not v.strip():
                raise ValueError('Query text must not be empty')
            return v