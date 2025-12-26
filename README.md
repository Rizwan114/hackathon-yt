# Book-Based AI Chatbot

This project implements a book-based chatbot that allows users to ask questions about a specific book and receive accurate answers based only on the book's content. The system uses a RAG (Retrieval-Augmented Generation) approach with Qdrant vector database, Cohere embeddings, and Gemini API.

## Features

- Book-specific Q&A: The chatbot only responds based on content from the specified book
- RAG (Retrieval-Augmented Generation): Uses vector search to find relevant content before generating responses
- Web interface: Docusaurus-based frontend for easy interaction
- API endpoints: FastAPI backend with comprehensive endpoints for chat, retrieval, and validation
- Session management: Maintains conversation context
- Source attribution: Shows where in the book the information comes from

## Architecture

- **Backend**: Python/FastAPI with agents framework
- **Frontend**: Docusaurus v3/React
- **Vector Database**: Qdrant Cloud
- **Embeddings**: Cohere
- **LLM**: Google Gemini via OpenAI-compatible interface
- **Deployment**: Vercel-ready configuration

## Setup

### Prerequisites

- Python 3.14+
- Node.js 20+
- uv package manager

### Backend Setup

1. Navigate to the backend directory:
```bash
cd backend
```

2. Install dependencies:
```bash
uv pip install -r requirements.txt
```

3. Create environment file:
```bash
cp .env.example .env
```

4. Add your API keys to `.env`:
```env
GEMINI_API_KEY=your_gemini_api_key_here
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

5. Start the backend server:
```bash
uv run main.py
```

### Frontend Setup

1. Navigate to the website directory:
```bash
cd my-website
```

2. Install dependencies:
```bash
npm install
```

3. Start the development server:
```bash
npm start
```

## API Endpoints

- `POST /chat` - Main chat endpoint for book-based Q&A
- `POST /retrieve` - Retrieve relevant book content based on query
- `POST /validate` - Validate text against book content
- `POST /sessions` - Create a new chat session
- `GET /health` - Health check endpoint

## Deployment

The application is ready for deployment on Vercel. The configuration is provided in `vercel.json`.

## Technologies Used

- Python 3.14
- FastAPI
- Cohere API
- Qdrant Vector Database
- Google Gemini API
- Docusaurus v3
- React
- Node.js

## Project Structure

```
├── backend/                 # Python backend with FastAPI
│   ├── main.py             # Main API server
│   ├── front.py            # Agent-based chatbot
│   ├── api/                # API routes
│   ├── models/             # Data models
│   ├── schemas/            # Pydantic schemas
│   ├── services/           # Business logic
│   └── rag/                # RAG components
├── my-website/            # Docusaurus frontend
│   ├── src/                # Source files
│   │   ├── components/     # React components
│   │   ├── pages/          # Page components
│   │   └── services/       # Frontend services
│   └── docs/               # Documentation
├── docs/                  # Additional documentation
└── specs/                 # Feature specifications
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

[Specify your license here]