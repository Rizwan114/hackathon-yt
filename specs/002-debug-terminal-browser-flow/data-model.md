# Data Model: Debug Terminal vs Browser Flow Issue

## Overview

This document outlines the data models and entities relevant to fixing the browser vs terminal flow issue in the book-based chatbot. The focus is on ensuring consistent data flow between both interfaces.

## Core Entities

### User Query
- **Description**: Input from user that needs to be processed using book content, regardless of interface
- **Fields**:
  - `query_text` (string): The actual query text from the user
  - `interface_type` (string): Either "terminal" or "browser" for debugging purposes
  - `timestamp` (datetime): When the query was submitted
  - `session_id` (string): Session identifier for tracking conversation context
  - `user_id` (string, optional): User identifier if authenticated

### Interface Context
- **Description**: Information about whether request originated from terminal or browser, used for debugging
- **Fields**:
  - `interface_type` (string): Either "terminal" or "browser"
  - `request_headers` (object): HTTP headers from the request (browser only)
  - `request_origin` (string): Origin of the request (browser only)
  - `client_info` (object): Information about the client making the request

### Response Payload
- **Description**: Data returned to user containing book content-based answers
- **Fields**:
  - `response_text` (string): The response text containing book-based answers
  - `book_sources` (array): List of book sources used to generate the response
  - `confidence_score` (number): Confidence level in the book-based response
  - `timestamp` (datetime): When the response was generated
  - `query_id` (string): Reference to the original query

### Book Content
- **Description**: Source material used to generate responses, must be accessible from both interfaces
- **Fields**:
  - `content_id` (string): Unique identifier for the book content
  - `title` (string): Title of the book or content
  - `text` (string): The actual book content text
  - `embedding` (array): Vector embedding for similarity search
  - `metadata` (object): Additional metadata about the content (author, chapter, etc.)

## API Request/Response Models

### Query Request Model
```json
{
  "query": "What is the main theme of the book?",
  "interface_context": {
    "type": "browser",
    "session_id": "session_123"
  }
}
```

### Query Response Model
```json
{
  "response": "The main theme of the book is...",
  "sources": ["chapter_1", "chapter_3"],
  "book_content_used": true,
  "confidence": 0.95
}
```

## Data Flow Patterns

### Terminal Request Flow
1. Terminal client sends query directly to backend
2. Backend processes query with book content context
3. Response returned with book-based answers

### Browser Request Flow
1. Browser client sends query with CORS headers to backend
2. Backend processes query with book content context
3. Response returned with book-based answers

## Validation Rules

- All queries must contain non-empty query text
- Interface context must be properly set to track source
- Response must include book content sources when book-based answers are provided
- Request headers must be properly validated for browser requests