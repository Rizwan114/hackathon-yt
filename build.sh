#!/bin/bash

# Build script for book chatbot backend and frontend integration

echo "Starting build process for book chatbot..."

# Install backend dependencies
echo "Installing backend dependencies..."
pip install -r requirements.txt

# Check if frontend directory exists and has package.json
if [ -d "frontend" ] && [ -f "frontend/package.json" ]; then
    echo "Building frontend application..."
    cd frontend
    npm install
    npm run build
    cd ..
else
    echo "Frontend directory not found or package.json missing. Skipping frontend build."
fi

echo "Build process completed!"
echo ""
echo "To run the backend server:"
echo "  uvicorn app:app --host 0.0.0.0 --port 8000"
echo ""
echo "API documentation available at:"
echo "  http://localhost:8000/docs (Swagger UI)"
echo "  http://localhost:8000/redoc (ReDoc)"