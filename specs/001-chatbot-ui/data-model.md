# Data Model: Floating Chatbot UI

## Entities

### ChatMessage
Represents a single message in the conversation.

**Fields**:
- `id: string` - Unique identifier for the message
- `sender: 'user' | 'bot'` - Indicates the sender of the message
- `content: string` - The text content of the message
- `timestamp: Date` - When the message was created/sent
- `status: 'sent' | 'delivered' | 'read'` - Message delivery status (for future use)

**Validation rules**:
- `content` must be non-empty string
- `content` length must be less than 1000 characters
- `sender` must be either 'user' or 'bot'

### ChatSession
Represents the current chat session state.

**Fields**:
- `id: string` - Unique identifier for the session
- `messages: ChatMessage[]` - Array of messages in the conversation
- `isOpen: boolean` - Whether the chat interface is currently open
- `isTyping: boolean` - Whether the bot is currently "typing" a response
- `lastActive: Date` - Timestamp of last interaction

**Validation rules**:
- `messages` array length must be less than 1000
- `isOpen` must be boolean
- `isTyping` must be boolean

### ChatConfig
Configuration options for the chatbot component.

**Fields**:
- `botName: string` - Display name for the bot (default: "Assistant")
- `headerColor: string` - Color for the chat header (default: "#007cba")
- `position: { bottom: string, right: string }` - CSS position values
- `maxMessages: number` - Maximum number of messages to display (default: 50)

**Validation rules**:
- `botName` must be non-empty string
- `headerColor` must be valid CSS color
- `position` values must be valid CSS position values
- `maxMessages` must be positive integer

## State Transitions

### ChatSession State Transitions
- `initial` → `open`: When user clicks the floating chat button
- `open` → `closed`: When user clicks the close button or minimizes
- `open` → `typing`: When user sends message and bot is preparing response
- `typing` → `open`: When bot response is ready to display
- `open` → `closed`: When user closes the chat interface

## Relationships
- One `ChatSession` contains many `ChatMessage` objects
- `ChatMessage` belongs to one `ChatSession`
- `ChatConfig` applies to one `ChatSession`