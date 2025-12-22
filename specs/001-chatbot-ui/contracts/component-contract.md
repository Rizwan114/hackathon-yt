# Chatbot Component API Contract

## Component Interface

### Chatbot Props
```typescript
interface ChatbotProps {
  /**
   * Configuration options for the chatbot
   */
  config?: ChatConfig;

  /**
   * Optional callback when a message is sent
   */
  onMessageSend?: (message: string) => void;

  /**
   * Optional callback when bot responds
   */
  onBotResponse?: (response: string) => void;
}
```

### ChatConfig Interface
```typescript
interface ChatConfig {
  /**
   * Display name for the bot
   * @default "Assistant"
   */
  botName?: string;

  /**
   * Color for the chat header
   * @default "#007cba"
   */
  headerColor?: string;

  /**
   * CSS position values
   * @default { bottom: "20px", right: "20px" }
   */
  position?: {
    bottom: string;
    right: string;
  };

  /**
   * Maximum number of messages to display
   * @default 50
   */
  maxMessages?: number;
}
```

## Component Behavior

### Public Methods
- `sendMessage(text: string)`: Send a message to the chatbot
- `clearHistory()`: Clear the message history
- `toggleChat()`: Open/close the chat interface

### Events
- `onMessageSent`: Emitted when user sends a message
- `onResponseReceived`: Emitted when bot responds
- `onChatToggled`: Emitted when chat interface is opened/closed

## Response Format
The component will return a React element with the following structure:
```tsx
<div className="floating-chatbot">
  <div className="chatbot-button" onClick={toggleChat}>
    {/* Floating button content */}
  </div>
  <div className="chatbot-container" style={positionStyle}>
    {/* Chat interface content */}
  </div>
</div>
```