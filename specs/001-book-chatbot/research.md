# Research: Book-Based Chatbot Integration

## Decision: Project Structure
**Rationale**: The project has a clear separation between backend (Python/FastAPI) and frontend (Docusaurus React). The backend contains the RAG system with Qdrant vector database and Cohere embeddings. The frontend is a Docusaurus site that will need to communicate with the backend API.

## Decision: Current Implementation Status
**Rationale**: The project already has significant functionality implemented:
- `backend/front.py` contains a working agent that connects to Qdrant and Cohere
- `backend/main.py` contains commented-out FastAPI endpoints for a full RAG system
- The system is designed to work with book content indexed in Qdrant
- Uses Gemini API via OpenAI-compatible interface

## Decision: Technology Stack
**Rationale**:
- Backend: Python with FastAPI, Cohere embeddings, Qdrant vector database, Gemini API
- Frontend: Docusaurus v3 with React
- The system uses openai-agents library for the chat interface

## Decision: Integration Approach
**Rationale**: The system needs to connect the existing agent in `front.py` with the Docusaurus frontend through API calls. The commented-out FastAPI endpoints in `main.py` provide a good foundation for the API layer.

## Decision: Book Content Integration
**Rationale**: The system already has mechanisms to extract content from URLs (as seen in main.py), chunk it, and store it in Qdrant. The existing sitemap URL suggests the book content is already available on a website.

## Alternatives Considered:
1. Complete rewrite vs. extending existing code: Chose to extend existing code as it's already functional
2. Different vector databases: Qdrant is already configured and working
3. Different LLM providers: Gemini is already configured via OpenAI-compatible interface
4. Different frontend frameworks: Docusaurus is already set up and working

## Known Issues Identified:
1. The `front.py` script fails due to rate limits on the Gemini API (not an implementation issue, but a quota issue)
2. The main.py file has commented-out FastAPI code that needs to be activated
3. No clear connection between the Docusaurus frontend and backend API
4. The build process needs to be tested with `npm run build` in the my-website directory