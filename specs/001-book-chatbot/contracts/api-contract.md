# API Contract: Book-Based Chatbot

## Chat Endpoint
```
POST /chat
```

### Request
```json
{
  "message": "string (user's question about the book)",
  "user_id": "string (user identifier)",
  "session_id": "string (optional, session identifier)",
  "module_id": "string (optional, default: 'module-1-ros2')",
  "top_k": "integer (optional, default: 5)",
  "min_similarity": "float (optional, default: 0.3)"
}
```

### Response
```json
{
  "response": "string (AI-generated answer based on book content)",
  "sources": [
    {
      "module": "string (module/chapter identifier)",
      "chapter": "string (chapter title)",
      "section": "string (section identifier)",
      "content": "string (relevant book content)",
      "similarity_score": "float (relevance score 0-1)"
    }
  ],
  "session_id": "string (session identifier)",
  "timestamp": "string (ISO format timestamp)"
}
```

### Error Responses
- 400: Invalid request format
- 429: Rate limit exceeded
- 500: Internal server error

---

## Retrieve Endpoint
```
POST /retrieve
```

### Request
```json
{
  "query": "string (search query for book content)",
  "module_id": "string (optional, default: 'module-1-ros2')",
  "top_k": "integer (optional, default: 5)",
  "min_similarity": "float (optional, default: 0.3)"
}
```

### Response
```json
{
  "results": [
    {
      "module": "string",
      "chapter": "string",
      "section": "string",
      "content": "string (retrieved book content)",
      "similarity_score": "float",
      "metadata": "object (additional metadata)"
    }
  ]
}
```

---

## Validation Endpoint
```
POST /validate
```

### Request
```json
{
  "text": "string (text to validate against book content)",
  "module_id": "string (optional, default: 'module-1-ros2')"
}
```

### Response
```json
{
  "valid": "boolean (whether text matches book content)",
  "matches": [
    {
      "source": "string (source location)",
      "similarity": "float (similarity score)",
      "content": "string (matching content excerpt)"
    }
  ]
}
```

---

## Session Management
```
POST /sessions
```

### Request
```json
{
  "user_id": "string",
  "initial_query": "string (optional)"
}
```

### Response
```json
{
  "session_id": "string",
  "user_id": "string",
  "created_at": "string (ISO format)",
  "message_count": "integer"
}
```

---

## Health Check
```
GET /health
```

### Response
```json
{
  "status": "string (e.g., 'healthy')",
  "service": "string (service name)",
  "timestamp": "string (ISO format)"
}
```