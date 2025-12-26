# Quickstart: Book-Based Chatbot

## Prerequisites
- Python 3.14+ with uv package manager
- Node.js 20+ for frontend
- GEMINI_API_KEY in .env file
- Access to Qdrant Cloud instance
- Cohere API key

## Setup Backend

1. **Install Python dependencies**:
```bash
cd backend
uv pip install -r requirements.txt  # if available, or install from pyproject.toml
```

2. **Set up environment variables**:
Create `.env` file in backend directory:
```
GEMINI_API_KEY=your_gemini_api_key_here
```

3. **Run the chatbot service**:
```bash
# For testing the agent directly
uv run backend/front.py

# For running the API server (uncomment code in main.py first)
uv run backend/main.py
```

## Setup Frontend

1. **Install frontend dependencies**:
```bash
cd my-website
npm install
```

2. **Run development server**:
```bash
npm run start
```

3. **Build for production**:
```bash
npm run build
```

## API Endpoints (when main.py is activated)

- `POST /chat` - Main chat endpoint for book-based Q&A
- `POST /retrieve` - Retrieve relevant book content
- `POST /validate` - Validate content against book
- `GET /health` - Health check

## Testing the Integration

1. First run the agent to test basic functionality:
```bash
uv run backend/front.py
```

2. Check that the Qdrant connection works and book content is indexed

3. Test the API endpoints once main.py is properly configured

4. Connect the frontend to the backend API

## Troubleshooting

- If you get rate limit errors from Gemini, check your API key quota
- If Qdrant connection fails, verify the URL and API key in the code
- If build fails, check that all dependencies are properly installed