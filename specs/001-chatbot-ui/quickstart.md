# Quickstart Guide: Floating Chatbot UI

## Development Setup

1. Ensure you have the Docusaurus project running locally
2. Install dependencies if not already installed:
   ```bash
   npm install
   ```

## Component Structure

The chatbot component is composed of:
- `src/components/Chatbot.tsx`: Main React component with TypeScript
- `src/components/chatbot.css`: Separate styling file
- `src/theme/Layout/index.tsx`: Integration point for global injection

## Basic Usage

The chatbot component will be automatically injected into all pages via the Layout component. No additional setup is required on individual pages.

## Key Features

- Fixed bottom-right position that remains visible while scrolling
- Expandable/collapsible interface
- Scrollable message history display
- Message input with send functionality
- Demo response system (no backend required initially)

## Configuration Options

The component can be configured via props (will be implemented when needed):
- Bot name display
- Header color
- Position adjustments
- Maximum message count

## Future Integration Points

Architecture is prepared for:
- Async AI API calls
- Vector search integration
- Database connections
- User authentication

## Testing

To test locally:
1. Start development server: `npm run start`
2. Navigate to any documentation page
3. Verify chatbot appears in bottom-right corner
4. Test expanding/collapsing functionality
5. Test sending messages and receiving demo responses

## Building

To build the project with the new component:
```bash
npm run build
```
The build should complete successfully with the chatbot component integrated.