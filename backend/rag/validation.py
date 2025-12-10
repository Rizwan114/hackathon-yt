"""
Quality Validation Script for Physical AI Book RAG System
This script performs quality validation on the RAG system including:
- Content grounding verification
- Retrieval accuracy testing
- Response quality assessment
- Hallucination detection
"""

import os
import json
from typing import List, Dict, Any, Tuple
from dataclasses import dataclass
from enum import Enum
import logging
from datetime import datetime
import asyncio
from openai import OpenAI
import numpy as np
from sklearn.metrics.pairwise import cosine_similarity

from .chat import get_conversation_manager
from .retrieval import get_content_retriever
from .embedding import get_embedding_service


class ValidationResult(Enum):
    """
    Enumeration for validation results
    """
    PASS = "pass"
    FAIL = "fail"
    WARNING = "warning"


@dataclass
class ValidationReport:
    """
    Data class for validation report
    """
    test_name: str
    result: ValidationResult
    details: Dict[str, Any]
    timestamp: datetime
    score: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary format"""
        return {
            'test_name': self.test_name,
            'result': self.result.value,
            'details': self.details,
            'timestamp': self.timestamp.isoformat(),
            'score': self.score
        }


class RAGValidator:
    """
    Validator for RAG system quality assessment
    """
    def __init__(self):
        # Initialize OpenAI client for validation tasks
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            raise ValueError("OPENAI_API_KEY environment variable is required")

        self.client = OpenAI(api_key=api_key)
        self.model = os.getenv('VALIDATION_MODEL', 'gpt-4o-mini')

        # Get RAG components
        self.conversation_manager = get_conversation_manager()
        self.retriever = get_content_retriever()
        self.embedding_service = get_embedding_service()

        # Set up logging
        self.logger = logging.getLogger(__name__)

        self.logger.info(f"RAG Validator initialized with model: {self.model}")

    def validate_content_grounding(self, query: str, response: str, sources: List[Dict[str, Any]]) -> ValidationReport:
        """
        Validate that the response is grounded in the provided sources

        Args:
            query: Original query
            response: AI-generated response
            sources: Retrieved sources used in response generation

        Returns:
            Validation report
        """
        self.logger.info(f"Validating content grounding for query: {query[:50]}...")

        try:
            # Count how many sources were actually referenced in the response
            referenced_sources = 0
            for source in sources:
                content_snippet = source.get('content', '')[:100]  # Get first 100 chars
                if content_snippet.lower() in response.lower():
                    referenced_sources += 1

            # Calculate grounding ratio
            grounding_ratio = referenced_sources / len(sources) if sources else 0

            # Use AI to assess grounding quality
            grounding_assessment = self._assess_grounding_with_ai(query, response, sources)

            details = {
                'query': query,
                'referenced_sources': referenced_sources,
                'total_sources': len(sources),
                'grounding_ratio': grounding_ratio,
                'ai_assessment': grounding_assessment
            }

            # Determine result based on grounding ratio and AI assessment
            if grounding_ratio >= 0.5 and grounding_assessment.get('grounded', True):
                result = ValidationResult.PASS
                score = grounding_ratio
            elif grounding_ratio >= 0.3 or grounding_assessment.get('partially_grounded', False):
                result = ValidationResult.WARNING
                score = grounding_ratio * 0.5
            else:
                result = ValidationResult.FAIL
                score = grounding_ratio

            return ValidationReport(
                test_name="Content Grounding Validation",
                result=result,
                details=details,
                timestamp=datetime.utcnow(),
                score=score
            )

        except Exception as e:
            self.logger.error(f"Error in content grounding validation: {e}")
            return ValidationReport(
                test_name="Content Grounding Validation",
                result=ValidationResult.FAIL,
                details={'error': str(e)},
                timestamp=datetime.utcnow(),
                score=0.0
            )

    def _assess_grounding_with_ai(self, query: str, response: str, sources: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Use AI to assess if the response is properly grounded in sources

        Args:
            query: Original query
            response: AI-generated response
            sources: Retrieved sources

        Returns:
            Assessment dictionary
        """
        # Prepare context for AI evaluation
        context = f"""
        Query: {query}

        Response: {response}

        Sources:
        """
        for i, source in enumerate(sources):
            context += f"\nSource {i+1}: {source.get('content', '')[:300]}..."

        evaluation_prompt = f"""
        Evaluate if the response is properly grounded in the provided sources.
        Consider:
        1. Does the response directly reference information from the sources?
        2. Are the claims in the response supported by the source content?
        3. Is there any information in the response that contradicts the sources?
        4. Is the response making claims not supported by the sources (hallucination)?

        Respond with a JSON object in this format:
        {{
            "grounded": true/false,
            "partially_grounded": true/false,
            "has_hallucinations": true/false,
            "confidence_score": 0.0-1.0,
            "explanation": "Brief explanation of your assessment"
        }}
        """

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are an evaluator assessing if AI responses are properly grounded in provided sources."},
                    {"role": "user", "content": f"{context}\n\n{evaluation_prompt}"}
                ],
                temperature=0.0,  # Deterministic responses for validation
                response_format={"type": "json_object"}
            )

            assessment = json.loads(response.choices[0].message.content)
            return assessment

        except Exception as e:
            self.logger.error(f"Error in AI grounding assessment: {e}")
            return {
                "grounded": False,
                "partially_grounded": True,
                "has_hallucinations": True,
                "confidence_score": 0.0,
                "explanation": f"Error in AI assessment: {str(e)}"
            }

    def validate_retrieval_accuracy(self, query: str, retrieved_chunks: List[Any], expected_answers: List[str] = None) -> ValidationReport:
        """
        Validate the accuracy of retrieved content

        Args:
            query: Original query
            retrieved_chunks: Chunks retrieved by the system
            expected_answers: Expected answers for the query (optional)

        Returns:
            Validation report
        """
        self.logger.info(f"Validating retrieval accuracy for query: {query[:50]}...")

        try:
            if not retrieved_chunks:
                return ValidationReport(
                    test_name="Retrieval Accuracy Validation",
                    result=ValidationResult.FAIL,
                    details={
                        'query': query,
                        'retrieved_count': 0,
                        'expected_answers_present': expected_answers is not None and len(expected_answers) == 0,
                        'explanation': 'No chunks retrieved'
                    },
                    timestamp=datetime.utcnow(),
                    score=0.0
                )

            # Calculate relevance scores
            query_embedding = self.embedding_service.create_embedding(query)
            relevance_scores = []

            for chunk in retrieved_chunks:
                chunk_embedding = chunk.get('embedding') or self.embedding_service.create_embedding(chunk.get('text', ''))
                if isinstance(chunk_embedding, list):
                    chunk_embedding = np.array(chunk_embedding).reshape(1, -1)

                if isinstance(query_embedding, list):
                    query_embedding_np = np.array(query_embedding).reshape(1, -1)

                similarity = cosine_similarity(query_embedding_np, chunk_embedding)[0][0]
                relevance_scores.append(float(similarity))

            avg_relevance = sum(relevance_scores) / len(relevance_scores) if relevance_scores else 0

            # Check if expected answers are present in retrieved content
            expected_found = 0
            if expected_answers:
                for expected in expected_answers:
                    for chunk in retrieved_chunks:
                        if expected.lower() in chunk.get('text', '').lower():
                            expected_found += 1
                            break

            expected_recall = expected_found / len(expected_answers) if expected_answers else 0

            details = {
                'query': query,
                'retrieved_count': len(retrieved_chunks),
                'avg_relevance_score': avg_relevance,
                'relevance_scores': relevance_scores,
                'expected_answers_count': len(expected_answers) if expected_answers else 0,
                'expected_answers_found': expected_found,
                'expected_recall': expected_recall
            }

            # Determine result based on relevance and recall
            if avg_relevance >= 0.7 and expected_recall >= 0.8:
                result = ValidationResult.PASS
                score = (avg_relevance + expected_recall) / 2
            elif avg_relevance >= 0.5 or expected_recall >= 0.5:
                result = ValidationResult.WARNING
                score = (avg_relevance + expected_recall) / 4
            else:
                result = ValidationResult.FAIL
                score = (avg_relevance + expected_recall) / 2

            return ValidationReport(
                test_name="Retrieval Accuracy Validation",
                result=result,
                details=details,
                timestamp=datetime.utcnow(),
                score=score
            )

        except Exception as e:
            self.logger.error(f"Error in retrieval accuracy validation: {e}")
            return ValidationReport(
                test_name="Retrieval Accuracy Validation",
                result=ValidationResult.FAIL,
                details={'error': str(e)},
                timestamp=datetime.utcnow(),
                score=0.0
            )

    def validate_response_quality(self, query: str, response: str, sources: List[Dict[str, Any]]) -> ValidationReport:
        """
        Validate the overall quality of the response

        Args:
            query: Original query
            response: AI-generated response
            sources: Retrieved sources

        Returns:
            Validation report
        """
        self.logger.info(f"Validating response quality for query: {query[:50]}...")

        try:
            # Use AI to assess response quality
            quality_assessment = self._assess_response_quality_with_ai(query, response, sources)

            details = {
                'query': query,
                'response_length': len(response),
                'source_count': len(sources),
                'ai_assessment': quality_assessment
            }

            # Determine result based on AI assessment
            confidence_score = quality_assessment.get('confidence_score', 0.0)

            if confidence_score >= 0.8:
                result = ValidationResult.PASS
                score = confidence_score
            elif confidence_score >= 0.6:
                result = ValidationResult.WARNING
                score = confidence_score
            else:
                result = ValidationResult.FAIL
                score = confidence_score

            return ValidationReport(
                test_name="Response Quality Validation",
                result=result,
                details=details,
                timestamp=datetime.utcnow(),
                score=score
            )

        except Exception as e:
            self.logger.error(f"Error in response quality validation: {e}")
            return ValidationReport(
                test_name="Response Quality Validation",
                result=ValidationResult.FAIL,
                details={'error': str(e)},
                timestamp=datetime.utcnow(),
                score=0.0
            )

    def _assess_response_quality_with_ai(self, query: str, response: str, sources: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Use AI to assess response quality

        Args:
            query: Original query
            response: AI-generated response
            sources: Retrieved sources

        Returns:
            Quality assessment dictionary
        """
        # Prepare context for AI evaluation
        context = f"""
        Query: {query}

        Response: {response}

        Sources:
        """
        for i, source in enumerate(sources):
            context += f"\nSource {i+1}: {source.get('content', '')[:300]}..."

        evaluation_prompt = f"""
        Evaluate the quality of the response to the query based on:
        1. Relevance: How relevant is the response to the query?
        2. Accuracy: How accurate is the information provided?
        3. Completeness: Does the response adequately address the query?
        4. Clarity: Is the response clear and well-structured?
        5. Grounding: Is the response properly grounded in the provided sources?

        Respond with a JSON object in this format:
        {{
            "relevance_score": 0.0-1.0,
            "accuracy_score": 0.0-1.0,
            "completeness_score": 0.0-1.0,
            "clarity_score": 0.0-1.0,
            "grounding_score": 0.0-1.0,
            "overall_quality_score": 0.0-1.0,
            "confidence_score": 0.0-1.0,
            "feedback": "Constructive feedback on the response quality"
        }}
        """

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are an evaluator assessing the quality of AI responses to user queries."},
                    {"role": "user", "content": f"{context}\n\n{evaluation_prompt}"}
                ],
                temperature=0.0,  # Deterministic responses for validation
                response_format={"type": "json_object"}
            )

            assessment = json.loads(response.choices[0].message.content)
            return assessment

        except Exception as e:
            self.logger.error(f"Error in AI quality assessment: {e}")
            return {
                "relevance_score": 0.0,
                "accuracy_score": 0.0,
                "completeness_score": 0.0,
                "clarity_score": 0.0,
                "grounding_score": 0.0,
                "overall_quality_score": 0.0,
                "confidence_score": 0.0,
                "feedback": f"Error in AI assessment: {str(e)}"
            }

    def run_comprehensive_validation(self, query: str, expected_answer: str = None) -> List[ValidationReport]:
        """
        Run comprehensive validation including grounding, retrieval accuracy, and response quality

        Args:
            query: Query to validate
            expected_answer: Expected answer for comparison (optional)

        Returns:
            List of validation reports
        """
        self.logger.info(f"Running comprehensive validation for query: {query[:50]}...")

        # Get response from the RAG system
        session = self.conversation_manager.start_conversation("validator", query)
        response = session.messages[-1].content if session.messages else ""

        # Get sources used in the response
        # Note: In a real implementation, sources would be tracked during generation
        # For now, we'll retrieve relevant sources separately
        retrieved_chunks = self.retriever.search_book_content(query, top_k=5)

        sources = []
        for chunk in retrieved_chunks:
            sources.append({
                'title': chunk.source_title,
                'section': chunk.source_section,
                'content': chunk.text,
                'similarity_score': chunk.similarity_score
            })

        # Run individual validations
        grounding_report = self.validate_content_grounding(query, response, sources)
        retrieval_report = self.validate_retrieval_accuracy(query, retrieved_chunks, [expected_answer] if expected_answer else None)
        quality_report = self.validate_response_quality(query, response, sources)

        return [grounding_report, retrieval_report, quality_report]

    def validate_user_selected_text(self, selected_text: str, module_id: str = "module-1-ros2") -> ValidationReport:
        """
        Validate user-selected text against book content for grounding verification

        Args:
            selected_text: Text selected by the user
            module_id: Module to search in

        Returns:
            Validation report
        """
        self.logger.info(f"Validating user-selected text: {selected_text[:50]}...")

        try:
            # Search for similar content in the book
            retrieved_chunks = self.retriever.search_book_content(selected_text, top_k=3)

            # Check for matches
            matches = []
            for chunk in retrieved_chunks:
                similarity = chunk.similarity_score
                if similarity >= 0.7:  # High similarity threshold
                    matches.append({
                        'source': f"{chunk.source_title} ({chunk.source_section})",
                        'similarity': similarity,
                        'content': chunk.text[:200] + "..." if len(chunk.text) > 200 else chunk.text
                    })

            details = {
                'selected_text': selected_text,
                'module_id': module_id,
                'matches_found': len(matches),
                'matches': matches,
                'is_valid': len(matches) > 0
            }

            if len(matches) > 0:
                result = ValidationResult.PASS
                score = max(m['similarity'] for m in matches) if matches else 0.0
            else:
                result = ValidationResult.FAIL
                score = 0.0

            return ValidationReport(
                test_name="User Selected Text Validation",
                result=result,
                details=details,
                timestamp=datetime.utcnow(),
                score=score
            )

        except Exception as e:
            self.logger.error(f"Error in user selected text validation: {e}")
            return ValidationReport(
                test_name="User Selected Text Validation",
                result=ValidationResult.FAIL,
                details={'error': str(e)},
                timestamp=datetime.utcnow(),
                score=0.0
            )


class ValidationRunner:
    """
    Runner for executing validation tests
    """
    def __init__(self):
        self.validator = RAGValidator()
        self.logger = logging.getLogger(__name__)

    def run_test_suite(self, test_queries: List[Dict[str, str]]) -> Dict[str, Any]:
        """
        Run a suite of validation tests

        Args:
            test_queries: List of queries with optional expected answers

        Returns:
            Comprehensive test results
        """
        self.logger.info(f"Running test suite with {len(test_queries)} queries")

        all_reports = []
        passing_tests = 0
        warning_tests = 0
        failing_tests = 0

        for i, test_case in enumerate(test_queries):
            query = test_case['query']
            expected_answer = test_case.get('expected_answer')

            self.logger.info(f"Running test {i+1}/{len(test_queries)}: {query[:50]}...")

            # Run comprehensive validation
            reports = self.validator.run_comprehensive_validation(query, expected_answer)

            all_reports.extend(reports)

            # Count results
            for report in reports:
                if report.result == ValidationResult.PASS:
                    passing_tests += 1
                elif report.result == ValidationResult.WARNING:
                    warning_tests += 1
                else:
                    failing_tests += 1

        # Calculate overall metrics
        total_tests = len(all_reports)
        total_score = sum(report.score for report in all_reports) / total_tests if total_tests > 0 else 0.0

        summary = {
            'total_tests_run': total_tests,
            'passing_tests': passing_tests,
            'warning_tests': warning_tests,
            'failing_tests': failing_tests,
            'success_rate': passing_tests / total_tests if total_tests > 0 else 0.0,
            'average_score': total_score,
            'timestamp': datetime.utcnow().isoformat(),
            'individual_reports': [report.to_dict() for report in all_reports]
        }

        return summary

    def generate_validation_report(self, summary: Dict[str, Any], output_file: str = None) -> str:
        """
        Generate a validation report in JSON format

        Args:
            summary: Test results summary
            output_file: Optional output file path

        Returns:
            JSON string of the report
        """
        report_json = json.dumps(summary, indent=2)

        if output_file:
            with open(output_file, 'w', encoding='utf-8') as f:
                f.write(report_json)

        return report_json


# Global instance for easy access
rag_validator = RAGValidator()
validation_runner = ValidationRunner()


def get_rag_validator() -> RAGValidator:
    """
    Get the global RAG validator instance

    Returns:
        RAGValidator instance
    """
    return rag_validator


def get_validation_runner() -> ValidationRunner:
    """
    Get the global validation runner instance

    Returns:
        ValidationRunner instance
    """
    return validation_runner


# Example usage
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Run RAG system quality validation')
    parser.add_argument('--query', type=str, help='Single query to validate')
    parser.add_argument('--test-file', type=str, help='JSON file with test queries')
    parser.add_argument('--output', type=str, help='Output file for validation report')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose logging')

    args = parser.parse_args()

    # Set up logging
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    runner = get_validation_runner()

    if args.query:
        # Validate single query
        reports = runner.validator.run_comprehensive_validation(args.query)
        for report in reports:
            print(f"{report.test_name}: {report.result.value} (Score: {report.score:.2f})")
            print(f"Details: {report.details}")
            print("-" * 50)

    elif args.test_file:
        # Load test queries from file
        with open(args.test_file, 'r', encoding='utf-8') as f:
            test_queries = json.load(f)

        # Run test suite
        summary = runner.run_test_suite(test_queries)

        # Generate report
        report = runner.generate_validation_report(summary, args.output)

        print(f"Validation completed. Success rate: {summary['success_rate']:.2%}")
        print(f"Average score: {summary['average_score']:.2f}")
        if args.output:
            print(f"Report saved to: {args.output}")

    else:
        # Run example validation
        print("Running example validation...")

        # Example test queries
        test_queries = [
            {
                "query": "What is the robotic nervous system?",
                "expected_answer": "The robotic nervous system refers to the distributed architecture for humanoid robot control using ROS 2."
            },
            {
                "query": "How does rclpy integrate with ROS 2?",
                "expected_answer": "rclpy is the Python client library for ROS 2 that enables creating nodes, publishers, subscribers, and actions."
            }
        ]

        summary = runner.run_test_suite(test_queries)
        report = runner.generate_validation_report(summary)

        print(f"Example validation completed. Success rate: {summary['success_rate']:.2%}")
        print(f"Average score: {summary['average_score']:.2f}")