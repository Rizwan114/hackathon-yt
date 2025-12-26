# Build Instructions for Frontend Integration

## Running the Backend

1. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Run the application:**
   ```bash
   uvicorn app:app --reload --host 0.0.0.0 --port 8000
   ```

## For Production Deployment

1. **Build the Docker container:**
   ```bash
   docker build -t book-chatbot-backend .
   ```

2. **Run the Docker container:**
   ```bash
   docker run -p 8000:8000 book-chatbot-backend
   ```

## Frontend Integration

When deploying together with a frontend application:

1. **For development with frontend:**
   If your frontend has a `package.json` with a build script, you can run:
   ```bash
   npm run build
   ```
   This will build the frontend assets.

2. **Environment variables for frontend build:**
   Make sure your frontend is configured to connect to the backend API.
   In your frontend's environment configuration, set:
   - REACT_APP_API_URL (for React apps) or similar variable to point to your backend
   - For local development: `http://localhost:8000`
   - For production: your deployed backend URL

3. **Example frontend build command:**
   ```bash
   # If you have a frontend in a separate directory
   cd frontend && npm install && npm run build
   ```

4. **For combined deployment:**
   You can use a deployment script like this:
   ```bash
   #!/bin/bash
   echo "Building backend..."
   pip install -r requirements.txt

   echo "Building frontend..."
   cd frontend && npm install && npm run build && cd ..

   echo "Starting backend..."
   uvicorn app:app --host 0.0.0.0 --port 8000
   ```

## API Documentation

The API is automatically documented with Swagger UI when running locally:
- Swagger UI: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc