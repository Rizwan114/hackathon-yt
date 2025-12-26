# Research: Debug Terminal vs Browser Flow Issue

## Summary

This research investigates why the terminal flow works but browser flow fails in the book-based chatbot, specifically focusing on the browser → backend → cloud → LLM → browser flow. The research examines the five key areas mentioned in the feature specification: CORS, frontend payload, backend endpoint, cloud context injection, and response parsing.

## Key Findings

### 1. CORS Issues
- **Problem**: Browser requests may be blocked by CORS policies when communicating with backend
- **Evidence**: Browser requests subject to same-origin policy, terminal requests bypass this
- **Solution**: Configure proper CORS headers in backend to allow browser requests

### 2. Frontend Payload Differences
- **Problem**: Browser and terminal may send requests with different formats/headers
- **Evidence**: Different HTTP clients may format requests differently
- **Solution**: Ensure browser frontend sends identical payload format as terminal

### 3. Backend Endpoint Handling
- **Problem**: Backend may handle requests differently based on origin or headers
- **Evidence**: Backend logic might have different paths for different request types
- **Solution**: Verify backend treats all requests identically regardless of source

### 4. Cloud Context Injection
- **Problem**: Book content context may not be properly injected for browser requests
- **Evidence**: Terminal flow works, suggesting context injection happens correctly there
- **Solution**: Ensure context injection mechanism works for all request sources

### 5. Response Parsing
- **Problem**: Browser may parse responses differently than terminal
- **Evidence**: Different client-side processing between browser and terminal
- **Solution**: Ensure response format is consistent and properly handled by browser

## Detailed Analysis

### Current Architecture
- **Frontend**: Browser interface in `my-website/` directory
- **Backend**: Logic in `backend/front.py` as specified by constitution
- **Cloud Router**: Existing infrastructure to be preserved
- **LLM**: Existing integration to be preserved

### Terminal Flow (Working)
1. Terminal sends query to backend
2. Backend processes query using book content
3. Cloud router and LLM integration work correctly
4. Response contains book-based answers

### Browser Flow (Failing)
1. Browser sends query to backend (may have CORS/payload issues)
2. Backend may process differently than terminal requests
3. Cloud context injection may not work properly
4. Response may not contain book-based answers

## Root Cause Hypothesis

The most likely root causes are:
1. **CORS misconfiguration** - Browser requests blocked by security policies
2. **Request header differences** - Backend processes browser vs terminal requests differently
3. **Context injection dependency** - Book content injection tied to specific request characteristics present in terminal but not browser

## Recommended Approach

1. **Verify CORS configuration** - Ensure backend allows browser requests
2. **Compare request formats** - Analyze differences between terminal and browser requests
3. **Debug backend processing** - Identify where browser requests diverge from terminal
4. **Validate context injection** - Confirm book content injection works for browser requests
5. **Test response handling** - Ensure browser properly processes responses

## Implementation Strategy

Focus on minimal changes to fix the browser flow while preserving:
- Existing cloud router infrastructure
- Current LLM integration
- Backend architecture in `backend/front.py`
- Terminal functionality

This aligns with the constitution requirements for minimal code changes and backend integrity.