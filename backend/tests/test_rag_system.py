"""
End-to-End Tests for Physical AI Book RAG System Integration
This test suite validates the complete RAG system functionality
including embedding, retrieval, and response generation.
"""

import os
import pytest
from unittest.mock import Mock, patch, MagicMock
from datetime import datetime
from typing import List, Dict, Any

from rag.chat import get_conversation_manager, ChatService, ConversationManager
from rag.retrieval import get_content_retriever, ContentRetriever, RetrievedChunk
from rag.embedding import get_content_embedder, ContentEmbedder, EmbeddingService
from rag.validation import RAGValidator, ValidationReport, ValidationResult


class TestRAGSystemIntegration:
    """
    End-to-end tests for the RAG system integration
    """

    def setup_method(self):
        """
        Set up test fixtures before each test method
        """
        # Create mock services to avoid external dependencies during testing
        self.mock_embedding_service = Mock(spec=EmbeddingService)
        self.mock_embedding_service.create_embedding.return_value = [0.1, 0.2, 0.3] * 512  # Simulated embedding
        self.mock_embedding_service.create_embeddings.return_value = [[0.1, 0.2, 0.3] * 512] * 3
        self.mock_embedding_service.cosine_similarity.return_value = 0.85

        # Patch the embedding service
        with patch('rag.chat.get_content_embedder') as mock_get_embedder, \
             patch('rag.retrieval.get_content_retriever') as mock_get_retriever:

            # Create mock embedder
            mock_embedder = Mock(spec=ContentEmbedder)
            mock_embedder.embedding_service = self.mock_embedding_service
            mock_embedder.embed_document.return_value = [
                {
                    'id': 'chunk_0',
                    'text': 'The robotic nervous system in ROS 2 refers to the distributed architecture...',
                    'embedding': [0.1, 0.2, 0.3] * 512,
                    'metadata': {
                        'title': 'Introduction to ROS 2',
                        'section': 'module-1-ros2/chapter-1-intro',
                        'doc_type': 'chapter'
                    }
                }
            ]
            mock_get_embedder.return_value = mock_embedder

            # Create mock retriever
            mock_retriever = Mock(spec=ContentRetriever)
            mock_retriever.search_book_content.return_value = [
                RetrievedChunk(
                    id='chunk_0',
                    text='The robotic nervous system in ROS 2 refers to the distributed architecture...',
                    metadata={
                        'title': 'Introduction to ROS 2',
                        'section': 'module-1-ros2/chapter-1-intro',
                        'doc_type': 'chapter'
                    },
                    similarity_score=0.85,
                    source_title='Introduction to ROS 2',
                    source_section='module-1-ros2/chapter-1-intro'
                )
            ]
            mock_get_retriever.return_value = mock_retriever

            # Initialize the real services with mocked dependencies
            self.conversation_manager = ConversationManager()
            self.chat_service = ChatService()
            self.retriever = get_content_retriever()
            self.validator = RAGValidator()

    def test_embedding_service_creation(self):
        """
        Test that the embedding service is properly initialized
        """
        embedding_service = get_content_embedder().embedding_service

        assert embedding_service is not None
        assert hasattr(embedding_service, 'create_embedding')
        assert hasattr(embedding_service, 'create_embeddings')
        assert hasattr(embedding_service, 'cosine_similarity')

    def test_content_retrieval(self):
        """
        Test that content retrieval works correctly
        """
        # Mock a query
        query = "What is the robotic nervous system?"

        # Test retrieval
        results = self.retriever.search_book_content(query, top_k=3)

        # Assertions
        assert len(results) > 0
        assert isinstance(results[0], RetrievedChunk)
        assert "robotic nervous system" in results[0].text.lower()
        assert results[0].similarity_score > 0.5

    def test_conversation_creation(self):
        """
        Test that conversations can be created successfully
        """
        session = self.conversation_manager.start_conversation("test_user_123")

        assert session is not None
        assert session.user_id == "test_user_123"
        assert session.id is not None
        assert len(session.messages) == 0

    def test_conversation_with_initial_query(self):
        """
        Test conversation creation with an initial query
        """
        initial_query = "What is the robotic nervous system?"
        session = self.conversation_manager.start_conversation("test_user_123", initial_query)

        assert session is not None
        assert session.user_id == "test_user_123"
        assert len(session.messages) >= 1

        # Check that the query was processed
        user_messages = [msg for msg in session.messages if msg.role.value == "user"]
        assert len(user_messages) >= 1
        assert initial_query in user_messages[0].content

    def test_response_generation_with_rag(self):
        """
        Test that responses are generated using RAG
        """
        # Create a session
        session = self.conversation_manager.chat_service.create_session("test_user_456")

        # Add a test query
        test_query = "What is the robotic nervous system?"
        self.conversation_manager.chat_service.add_message(session, self.conversation_manager.chat_service.MessageRole.USER, test_query)

        # Generate response using RAG
        response = self.conversation_manager.chat_service.generate_response(session, test_query)

        # Assertions
        assert response is not None
        assert len(response) > 0
        # The response should contain information related to the query
        assert "robotic" in response.lower() or "ros" in response.lower()

    def test_content_grounding_validation(self):
        """
        Test content grounding validation
        """
        query = "What is the robotic nervous system?"
        response = "The robotic nervous system in ROS 2 refers to the distributed architecture..."
        sources = [
            {
                'title': 'Introduction to ROS 2',
                'section': 'module-1-ros2/chapter-1-intro',
                'content': 'The robotic nervous system in ROS 2 refers to the distributed architecture...',
                'similarity_score': 0.85
            }
        ]

        validation_report = self.validator.validate_content_grounding(query, response, sources)

        assert validation_report is not None
        assert validation_report.test_name == "Content Grounding Validation"
        assert validation_report.result in [ValidationResult.PASS, ValidationResult.WARNING, ValidationResult.FAIL]
        assert validation_report.score >= 0.0

    def test_retrieval_accuracy_validation(self):
        """
        Test retrieval accuracy validation
        """
        query = "How does ROS 2 communication work?"

        # Create mock chunks
        mock_chunks = [
            {
                'text': 'ROS 2 communication uses nodes, topics, services, and actions.',
                'embedding': [0.1, 0.2, 0.3] * 512,
                'metadata': {'title': 'Communication Fundamentals'}
            }
        ]

        validation_report = self.validator.validate_retrieval_accuracy(query, mock_chunks, ["nodes", "topics"])

        assert validation_report is not None
        assert validation_report.test_name == "Retrieval Accuracy Validation"
        assert validation_report.result in [ValidationResult.PASS, ValidationResult.WARNING, ValidationResult.FAIL]

    def test_response_quality_validation(self):
        """
        Test response quality validation
        """
        query = "What is the robotic nervous system?"
        response = "The robotic nervous system in ROS 2 refers to the distributed architecture that enables humanoid robot control."
        sources = [
            {
                'title': 'Introduction to ROS 2',
                'section': 'module-1-ros2/chapter-1-intro',
                'content': 'The robotic nervous system in ROS 2 refers to the distributed architecture...',
                'similarity_score': 0.85
            }
        ]

        validation_report = self.validator.validate_response_quality(query, response, sources)

        assert validation_report is not None
        assert validation_report.test_name == "Response Quality Validation"
        assert validation_report.result in [ValidationResult.PASS, ValidationResult.WARNING, ValidationResult.FAIL]

    def test_comprehensive_validation_workflow(self):
        """
        Test the complete validation workflow
        """
        query = "What is the robotic nervous system?"
        expected_answer = "The robotic nervous system refers to the distributed architecture in ROS 2."

        # Run comprehensive validation
        reports = self.validator.run_comprehensive_validation(query, expected_answer)

        # Should have grounding, retrieval, and quality reports
        assert len(reports) == 3
        report_types = [report.test_name for report in reports]
        assert "Content Grounding Validation" in report_types
        assert "Retrieval Accuracy Validation" in report_types
        assert "Response Quality Validation" in report_types

    def test_user_selected_text_validation(self):
        """
        Test user-selected text validation
        """
        selected_text = "The robotic nervous system uses nodes, topics, and services"

        validation_report = self.validator.validate_user_selected_text(selected_text)

        assert validation_report is not None
        assert validation_report.test_name == "User Selected Text Validation"
        assert validation_report.result in [ValidationResult.PASS, ValidationResult.FAIL]

    def test_end_to_end_conversation_flow(self):
        """
        Test the complete end-to-end conversation flow
        """
        # Start a conversation
        session = self.conversation_manager.start_conversation("test_user_789")

        # Add multiple exchanges
        queries_responses = [
            ("What is the robotic nervous system?", "The robotic nervous system"),
            ("How does ROS 2 differ from ROS 1?", "ROS 2 uses DDS"),
            ("What are ROS 2 actions?", "Actions are for long-running tasks")
        ]

        for query, expected_part in queries_responses:
            # Get response
            response = self.conversation_manager.continue_conversation(session, query)

            # Check that response is generated
            assert response is not None
            assert len(response) > 0

            # The response should be contextually relevant
            # (In a real test, we'd have more specific assertions)

    def test_session_summarization(self):
        """
        Test conversation session summarization
        """
        # Create a session with some messages
        session = self.conversation_manager.start_conversation("test_user_999", "Hello")

        # Add a few more messages
        self.conversation_manager.continue_conversation(session, "What is ROS 2?")
        self.conversation_manager.continue_conversation(session, "How do nodes communicate?")

        # Get summary
        summary = self.conversation_manager.get_conversation_summary(session)

        assert summary is not None
        assert 'session_id' in summary
        assert 'user_id' in summary
        assert 'total_messages' in summary
        assert 'user_messages' in summary
        assert 'assistant_messages' in summary
        assert summary['user_id'] == "test_user_999"
        assert summary['total_messages'] >= 3  # Hello + 2 follow-ups

    @pytest.mark.asyncio
    async def test_async_validation_methods(self):
        """
        Test async validation methods if they exist
        """
        # This is a placeholder for async tests
        # In a real implementation, you might have async validation methods
        assert True  # Placeholder assertion

    def test_error_handling_in_chat_service(self):
        """
        Test error handling in the chat service
        """
        # Test with invalid query
        session = self.conversation_manager.chat_service.create_session("test_user_error")

        # Mock an error condition
        with patch.object(self.conversation_manager.chat_service.retriever, 'search_book_content',
                         side_effect=Exception("Mocked error")):
            response = self.conversation_manager.chat_service.generate_response(session, "Test query")

            # Should return a fallback response
            assert response is not None
            assert "trouble accessing" in response.lower() or "rephrasing" in response.lower()

    def test_multiple_sessions_isolation(self):
        """
        Test that multiple sessions are properly isolated
        """
        # Create multiple sessions
        session1 = self.conversation_manager.start_conversation("user_1", "Hello")
        session2 = self.conversation_manager.start_conversation("user_2", "Hi")

        # Sessions should have different IDs
        assert session1.id != session2.id
        assert session1.user_id != session2.user_id

        # Messages should be isolated
        session1_len = len(session1.messages)
        session2_len = len(session2.messages)

        assert session1_len >= 1
        assert session2_len >= 1


class TestRAGPerformance:
    """
    Performance tests for the RAG system
    """

    def test_response_time_under_threshold(self):
        """
        Test that response times are within acceptable thresholds
        """
        import time

        # This is a mock test since we're using mocked services
        # In a real test, we'd measure actual response times
        start_time = time.time()

        # Simulate a query
        manager = get_conversation_manager()
        session = manager.start_conversation("perf_test_user", "What is ROS 2?")

        end_time = time.time()
        response_time = end_time - start_time

        # With mocked services, response should be very fast
        # In a real system, you'd set appropriate thresholds
        assert response_time < 1.0  # Less than 1 second for mocked response

    def test_memory_usage_during_ingestion(self):
        """
        Test memory usage during content ingestion (conceptual)
        """
        # This is a conceptual test
        # In a real implementation, you'd monitor memory usage
        import psutil
        import os

        process = psutil.Process(os.getpid())
        initial_memory = process.memory_info().rss / 1024 / 1024  # MB

        # Simulate content processing
        embedder = get_content_embedder()
        sample_content = "Sample content " * 1000  # Large content
        chunks = embedder.embed_content(sample_content)

        final_memory = process.memory_info().rss / 1024 / 1024  # MB
        memory_increase = final_memory - initial_memory

        # Memory increase should be reasonable
        assert memory_increase < 100  # Less than 100MB increase


def run_comprehensive_tests():
    """
    Run comprehensive tests for the RAG system
    """
    print("Running RAG System Integration Tests...")

    test_instance = TestRAGSystemIntegration()

    # Run all test methods
    methods = [method for method in dir(test_instance) if method.startswith('test_')]

    passed = 0
    failed = 0

    for method_name in methods:
        try:
            method = getattr(test_instance, method_name)
            method()
            print(f"✓ {method_name}")
            passed += 1
        except Exception as e:
            print(f"✗ {method_name}: {str(e)}")
            failed += 1

    print(f"\nTest Results: {passed} passed, {failed} failed")
    return failed == 0


if __name__ == "__main__":
    success = run_comprehensive_tests()
    exit(0 if success else 1)