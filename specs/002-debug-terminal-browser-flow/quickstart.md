# Quickstart Guide: Debug Terminal vs Browser Flow Issue

## Overview
This guide provides instructions for setting up and debugging the book-based chatbot to ensure consistent behavior between terminal and browser interfaces.

## Prerequisites
- Python 3.8+
- Node.js (for browser interface)
- Existing cloud router and LLM configuration
- Book content data

## Setup Instructions

### 1. Backend Setup
```bash
cd backend
pip install -r requirements.txt  # if exists
# Or install required packages manually:
pip install fastapi uvicorn
```

### 2. Frontend Setup
```bash
cd my-website
npm install  # if package.json exists
```

### 3. Environment Configuration
```bash
# Copy the example environment file
cp backend/.env.example backend/.env
# Update with your configuration values
```

## Testing Both Interfaces

### Terminal Interface (Working)
```bash
# Run terminal tests to confirm current functionality
python -c "import backend.front; print('Terminal interface working')"
# Or run your existing terminal test suite
```

### Browser Interface (Needs Fixing)
```bash
# Start the backend server
cd backend
uvicorn main:app --reload

# In another terminal, start the frontend
cd my-website
npm run dev  # or appropriate command for your setup
```

## Debugging Steps

### 1. Check CORS Configuration
- Verify that backend allows requests from browser origin
- Check `backend/front.py` for CORS settings
- Ensure browser requests aren't blocked

### 2. Compare Request Formats
- Monitor requests from both terminal and browser
- Check headers, payload format, and authentication
- Ensure consistency between interfaces

### 3. Verify Book Content Injection
- Check that book content is properly injected for all requests
- Verify the context injection mechanism works for browser requests
- Confirm book data is accessible from both interfaces

### 4. Test Response Processing
- Verify that responses contain book-based content in both interfaces
- Check response parsing in browser interface
- Ensure consistency in response format

## API Endpoints

### Chat Endpoint
- **URL**: `POST /chat`
- **Request Body**:
```json
{
  "query": "Your question here",
  "interface_type": "browser"
}
```
- **Response**:
```json
{
  "response": "Book-based answer",
  "sources": ["source1", "source2"],
  "book_content_used": true,
  "confidence": 0.95
}
```

## Troubleshooting

### Common Issues
1. **CORS Errors**: Check backend CORS configuration
2. **Different Response Formats**: Verify response parsing in browser
3. **Missing Book Content**: Confirm context injection for browser requests
4. **Authentication Issues**: Ensure consistent auth between interfaces

### Debugging Commands
```bash
# Check backend logs
tail -f backend/logs/app.log

# Monitor browser console for errors
# Use browser dev tools to inspect network requests
```

## Next Steps
1. Implement fixes based on debugging findings
2. Test both interfaces after changes
3. Verify book content is used consistently
4. Ensure no regression in terminal functionality