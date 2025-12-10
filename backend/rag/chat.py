"""
Chat Module for Physical AI Book RAG System
This module handles the conversation interface, managing chat sessions,
processing user queries, retrieving relevant content, and generating responses.
"""

import os
import uuid
from datetime import datetime
from typing import List, Dict, Any, Optional
from enum import Enum
from dataclasses import dataclass
from openai import OpenAI
from .retrieval import get_content_retriever, RetrievedChunk
from .embedding import get_content_embedder
import logging


class MessageRole(Enum):
    """
    Role of a message in the conversation
    """
    SYSTEM = "system"
    USER = "user"
    ASSISTANT = "assistant"


@dataclass
class ChatMessage:
    """
    Data class representing a chat message
    """
    id: str
    role: MessageRole
    content: str
    timestamp: datetime
    metadata: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary format"""
        return {
            'id': self.id,
            'role': self.role.value,
            'content': self.content,
            'timestamp': self.timestamp.isoformat(),
            'metadata': self.metadata or {}
        }


@dataclass
class ChatSession:
    """
    Data class representing a chat session
    """
    id: str
    user_id: str
    created_at: datetime
    updated_at: datetime
    messages: List[ChatMessage]
    metadata: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary format"""
        return {
            'id': self.id,
            'user_id': self.user_id,
            'created_at': self.created_at.isoformat(),
            'updated_at': self.updated_at.isoformat(),
            'messages': [msg.to_dict() for msg in self.messages],
            'metadata': self.metadata or {}
        }


class ChatService:
    """
    Service class for handling chat conversations with RAG capabilities
    """
    def __init__(self):
        # Initialize OpenAI client
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            raise ValueError("OPENAI_API_KEY environment variable is required")

        self.client = OpenAI(api_key=api_key)
        self.model = os.getenv('CHAT_MODEL', 'gpt-4o-mini')

        # Get retrieval service
        self.retriever = get_content_retriever()

        # Set up logging
        self.logger = logging.getLogger(__name__)

        self.logger.info(f"Chat service initialized with model: {self.model}")

    def create_session(self, user_id: str, initial_metadata: Optional[Dict[str, Any]] = None) -> ChatSession:
        """
        Create a new chat session

        Args:
            user_id: ID of the user starting the session
            initial_metadata: Initial metadata for the session

        Returns:
            New chat session
        """
        session_id = str(uuid.uuid4())
        now = datetime.utcnow()

        session = ChatSession(
            id=session_id,
            user_id=user_id,
            created_at=now,
            updated_at=now,
            messages=[],
            metadata=initial_metadata or {}
        )

        self.logger.info(f"Created new chat session: {session_id} for user: {user_id}")
        return session

    def add_message(self, session: ChatSession, role: MessageRole, content: str,
                   metadata: Optional[Dict[str, Any]] = None) -> ChatMessage:
        """
        Add a message to the chat session

        Args:
            session: Chat session to add message to
            role: Role of the message (user, assistant, system)
            content: Content of the message
            metadata: Additional metadata for the message

        Returns:
            Added chat message
        """
        message_id = str(uuid.uuid4())
        timestamp = datetime.utcnow()

        message = ChatMessage(
            id=message_id,
            role=role,
            content=content,
            timestamp=timestamp,
            metadata=metadata or {}
        )

        session.messages.append(message)
        session.updated_at = timestamp

        self.logger.debug(f"Added message to session {session.id}: {role.value} - {content[:50]}...")
        return message

    def get_conversation_history(self, session: ChatSession, limit: int = 10) -> List[ChatMessage]:
        """
        Get recent conversation history

        Args:
            session: Chat session to get history from
            limit: Maximum number of messages to return

        Returns:
            List of recent messages
        """
        return session.messages[-limit:]

    def process_query_with_rag(self, session: ChatSession, user_query: str,
                              top_k: int = 5, min_similarity: float = 0.3) -> str:
        """
        Process user query using RAG (Retrieval-Augmented Generation)

        Args:
            session: Current chat session
            user_query: User's query
            top_k: Number of relevant chunks to retrieve
            min_similarity: Minimum similarity threshold

        Returns:
            Generated response incorporating retrieved content
        """
        try:
            # Retrieve relevant content based on the query
            self.logger.info(f"Retrieving content for query: {user_query[:50]}...")

            retrieved_chunks = self.retriever.search_book_content(
                query=user_query,
                top_k=top_k,
                min_similarity=min_similarity
            )

            self.logger.info(f"Retrieved {len(retrieved_chunks)} relevant chunks")

            # Prepare context from retrieved content
            context_texts = []
            for chunk in retrieved_chunks:
                context_text = f"""
                Source: {chunk.source_title} ({chunk.source_section})
                Content: {chunk.text}
                Relevance Score: {chunk.similarity_score:.3f}
                """
                context_texts.append(context_text)

            # Create system message with context
            system_message = self._create_system_message(context_texts)

            # Get conversation history for context
            conversation_history = self.get_conversation_history(session, limit=10)
            conversation_messages = [
                {"role": msg.role.value, "content": msg.content}
                for msg in conversation_history
            ]

            # Prepare messages for OpenAI API
            messages = [
                {"role": "system", "content": system_message},
                *conversation_messages,
                {"role": "user", "content": user_query}
            ]

            # Call OpenAI API to generate response
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=0.7,
                max_tokens=1000
            )

            # Extract the response
            generated_response = response.choices[0].message.content

            # Log the interaction
            self.logger.info(f"Generated response for query: {user_query[:50]}...")

            return generated_response

        except Exception as e:
            self.logger.error(f"Error processing query with RAG: {str(e)}")
            # Fallback response if RAG fails
            return "I apologize, but I'm having trouble accessing the book content right now. Could you try rephrasing your question?"

    def _create_system_message(self, context_texts: List[str]) -> str:
        """
        Create system message with context for the AI model

        Args:
            context_texts: List of context texts retrieved

        Returns:
            Formatted system message
        """
        if not context_texts:
            return """
            You are an AI assistant helping users with questions about Physical AI and Humanoid Robotics.
            Answer questions based on your general knowledge while being helpful and accurate.
            """

        context_str = "\n\n".join(context_texts[:3])  # Use top 3 chunks to avoid exceeding token limits

        return f"""
        You are an AI assistant for the Physical AI Book. Your role is to help users understand
        concepts related to ROS 2, humanoid robotics, AI integration, and the robotic nervous system.

        Here is relevant content from the book that may help answer the user's question:
        {context_str}

        When answering, please:
        1. Reference the specific content from the book when possible
        2. Explain concepts clearly and accurately
        3. If the book content doesn't fully address the question, supplement with your general knowledge
        4. Be helpful, concise, and accurate
        5. If you can't find relevant information, acknowledge this politely
        6. Cite the source section when referring to book content
        """

    def generate_response(self, session: ChatSession, user_query: str) -> str:
        """
        Generate response to user query (wrapper for RAG processing)

        Args:
            session: Current chat session
            user_query: User's query

        Returns:
            Generated response
        """
        return self.process_query_with_rag(session, user_query)

    def get_relevant_sources(self, session: ChatSession, query: str, top_k: int = 3) -> List[RetrievedChunk]:
        """
        Get relevant sources for a query without generating a full response

        Args:
            session: Current chat session
            query: Query to search for
            top_k: Number of sources to return

        Returns:
            List of relevant content chunks
        """
        return self.retriever.search_book_content(query, top_k=top_k)


class ConversationManager:
    """
    High-level manager for handling complete conversations
    """
    def __init__(self):
        self.chat_service = ChatService()
        self.logger = logging.getLogger(__name__)

    def start_conversation(self, user_id: str, initial_query: Optional[str] = None) -> ChatSession:
        """
        Start a new conversation

        Args:
            user_id: ID of the user starting the conversation
            initial_query: Optional initial query to process

        Returns:
            New chat session with optional initial response
        """
        session = self.chat_service.create_session(user_id)

        if initial_query:
            # Add user's initial query to the session
            self.chat_service.add_message(session, MessageRole.USER, initial_query)

            # Generate and add AI response
            response = self.chat_service.generate_response(session, initial_query)
            self.chat_service.add_message(session, MessageRole.ASSISTANT, response)

        self.logger.info(f"Started conversation for user: {user_id}")
        return session

    def continue_conversation(self, session: ChatSession, user_message: str) -> str:
        """
        Continue an existing conversation

        Args:
            session: Existing chat session
            user_message: User's message

        Returns:
            AI-generated response
        """
        # Add user message to session
        self.chat_service.add_message(session, MessageRole.USER, user_message)

        # Generate response using RAG
        response = self.chat_service.generate_response(session, user_message)

        # Add AI response to session
        self.chat_service.add_message(session, MessageRole.ASSISTANT, response)

        self.logger.info(f"Continued conversation in session: {session.id}")
        return response

    def get_conversation_summary(self, session: ChatSession) -> Dict[str, Any]:
        """
        Get a summary of the conversation

        Args:
            session: Chat session to summarize

        Returns:
            Dictionary with conversation summary
        """
        total_messages = len(session.messages)
        user_messages = len([m for m in session.messages if m.role == MessageRole.USER])
        assistant_messages = len([m for m in session.messages if m.role == MessageRole.ASSISTANT])

        # Get last few messages for context
        recent_messages = [
            {
                'role': msg.role.value,
                'content': msg.content[:100] + "..." if len(msg.content) > 100 else msg.content,
                'timestamp': msg.timestamp.isoformat()
            }
            for msg in session.messages[-3:]  # Last 3 messages
        ]

        return {
            'session_id': session.id,
            'user_id': session.user_id,
            'created_at': session.created_at.isoformat(),
            'updated_at': session.updated_at.isoformat(),
            'total_messages': total_messages,
            'user_messages': user_messages,
            'assistant_messages': assistant_messages,
            'recent_messages': recent_messages
        }


# Global instance for easy access
chat_service = ChatService()
conversation_manager = ConversationManager()


def get_chat_service() -> ChatService:
    """
    Get the global chat service instance

    Returns:
        ChatService instance
    """
    return chat_service


def get_conversation_manager() -> ConversationManager:
    """
    Get the global conversation manager instance

    Returns:
        ConversationManager instance
    """
    return conversation_manager


# Example usage
if __name__ == "__main__":
    # Example of how to use the chat service
    manager = get_conversation_manager()

    # Start a new conversation
    session = manager.start_conversation("user_123", "What is the robotic nervous system?")

    print(f"Started session: {session.id}")
    print(f"Initial response: {session.messages[-1].content[:100]}...")

    # Continue the conversation
    response = manager.continue_conversation(session, "How does it relate to ROS 2?")
    print(f"Follow-up response: {response[:100]}...")

    # Get conversation summary
    summary = manager.get_conversation_summary(session)
    print(f"Conversation summary: {summary}")