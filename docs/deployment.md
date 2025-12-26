# Deployment Guide

This document provides instructions for deploying the book-based chatbot application.

## Prerequisites

- Node.js 20+ for frontend
- Python 3.14+ for backend
- Access to Qdrant Cloud instance
- Cohere API key
- Gemini API key

## Environment Configuration

Create a `.env` file in the `backend/` directory with the following variables:

```env
GEMINI_API_KEY=your_gemini_api_key_here
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

## Backend Deployment

1. Install Python dependencies:
```bash
cd backend
uv pip install -r requirements.txt
```

2. Start the backend server:
```bash
uv run main.py
```

The backend will be available at `http://localhost:8000`

## Frontend Deployment

1. Install frontend dependencies:
```bash
cd my-website
npm install
```

2. Build the frontend:
```bash
npm run build
```

3. Serve the frontend:
```bash
npm run serve
```

## Vercel Deployment

The application is configured for deployment on Vercel. The `vercel.json` file contains the necessary configuration for static hosting of the Docusaurus frontend.

1. Install the Vercel CLI:
```bash
npm install -g vercel
```

2. Deploy the application:
```bash
cd my-website
vercel
```

## Health Checks

The backend provides a health check endpoint at `/health` which returns the status of the service.

## Troubleshooting

- If you get API rate limit errors, check your API key quotas
- If the Qdrant connection fails, verify your URL and API key
- If the frontend can't connect to the backend, check CORS settings and URLs