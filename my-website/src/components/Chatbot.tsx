import React, { useState, useEffect, useRef } from 'react';
import './chatbot.css';

// Define TypeScript interfaces based on the data model
export interface ChatMessage {
  id: string;
  sender: 'user' | 'bot';
  content: string;
  timestamp: Date;
  status: 'sent' | 'delivered' | 'read';
}

export interface ChatSession {
  id: string;
  messages: ChatMessage[];
  isOpen: boolean;
  isTyping: boolean;
  lastActive: Date;
}

export interface ChatConfig {
  botName?: string;
  headerColor?: string;
  position?: {
    bottom: string;
    right: string;
  };
  maxMessages?: number;
}

// Default configuration
const DEFAULT_CONFIG: Required<ChatConfig> = {
  botName: 'Assistant',
  headerColor: '#007cba',
  position: { bottom: '20px', right: '20px' },
  maxMessages: 50,
};

// Props interface for the Chatbot component
interface ChatbotProps {
  config?: ChatConfig;
  onMessageSend?: (message: string) => void;
  onBotResponse?: (response: string) => void;
}

const Chatbot: React.FC<ChatbotProps> = ({
  config = {},
  onMessageSend,
  onBotResponse
}) => {
  // Merge default config with provided config
  const mergedConfig = {
    botName: config.botName ?? DEFAULT_CONFIG.botName,
    headerColor: config.headerColor ?? DEFAULT_CONFIG.headerColor,
    position: { ...DEFAULT_CONFIG.position, ...config.position },
    maxMessages: config.maxMessages ?? DEFAULT_CONFIG.maxMessages,
  };

  // State management
  const [session, setSession] = useState<ChatSession>({
    id: 'chat-session-1',
    messages: [],
    isOpen: false,
    isTyping: false,
    lastActive: new Date(),
  });

  const [inputValue, setInputValue] = useState<string>('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Validation functions
  const validateMessage = (content: string): boolean => {
    return content.trim().length > 0 && content.length < 1000;
  };

  const validateSender = (sender: string): sender is 'user' | 'bot' => {
    return sender === 'user' || sender === 'bot';
  };

  const validateMessageArray = (messages: ChatMessage[]): boolean => {
    return messages.length < 1000;
  };

  // Scroll to bottom of messages
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Effect to scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [session.messages]);

  // Function to add a message to the session
  const addMessage = (content: string, sender: 'user' | 'bot') => {
    if (!validateMessage(content) || !validateSender(sender)) {
      return;
    }

    const newMessage: ChatMessage = {
      id: `msg-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      sender,
      content,
      timestamp: new Date(),
      status: 'sent',
    };

    setSession(prev => ({
      ...prev,
      messages: [...prev.messages.slice(-(mergedConfig.maxMessages - 1)), newMessage],
      lastActive: new Date(),
    }));
  };

  // Handle sending a message
  const handleSendMessage = () => {
    if (!validateMessage(inputValue.trim())) {
      return;
    }

    // Add user message
    addMessage(inputValue.trim(), 'user');
    onMessageSend?.(inputValue.trim());

    // Clear input
    setInputValue('');

    // Simulate bot typing and response
    setSession(prev => ({ ...prev, isTyping: true }));

    // Simulate response after delay
    setTimeout(() => {
      // Generate demo response based on user input
      const response = generateDemoResponse(inputValue.trim());
      addMessage(response, 'bot');
      setSession(prev => ({ ...prev, isTyping: false }));
      onBotResponse?.(response);
    }, 1000 + Math.random() * 1000); // Random delay between 1-2 seconds
  };

  // Generate demo responses
  const generateDemoResponse = (userInput: string): string => {
    const lowerInput = userInput.toLowerCase();

    if (lowerInput.includes('hello') || lowerInput.includes('hi') || lowerInput.includes('hey')) {
      return 'Hello! How can I help you today?';
    } else if (lowerInput.includes('thank')) {
      return 'You\'re welcome! Is there anything else I can help with?';
    } else if (lowerInput.includes('help')) {
      return 'I\'m here to help! You can ask me questions about this documentation site.';
    } else if (lowerInput.includes('doc') || lowerInput.includes('document')) {
      return 'This documentation site contains information about the project. What specific topic would you like to know more about?';
    } else if (lowerInput.includes('example') || lowerInput.includes('how')) {
      return 'I can help explain examples! What would you like to see an example of?';
    } else if (lowerInput.includes('bye') || lowerInput.includes('goodbye')) {
      return 'Goodbye! Feel free to come back if you have more questions.';
    } else {
      // Random responses for variety
      const responses = [
        'That\'s an interesting question. Based on the documentation, I can tell you more about that.',
        'I understand you\'re asking about this topic. Let me explain it in more detail.',
        'Thanks for your question! Here\'s what I know about this subject.',
        'I can help with that. Here\'s what the documentation says about it.',
        'Great question! Here\'s the information you requested.',
        'I\'m happy to help explain that concept. Here\'s what you need to know.',
        'Based on the documentation, here\'s the answer to your question.',
        'Let me provide you with some information about that topic.'
      ];
      return responses[Math.floor(Math.random() * responses.length)];
    }
  };

  // Toggle chat open/close
  const toggleChat = () => {
    setSession(prev => ({
      ...prev,
      isOpen: !prev.isOpen,
      lastActive: new Date(),
    }));
  };

  // Close chat
  const closeChat = () => {
    setSession(prev => ({
      ...prev,
      isOpen: false,
      lastActive: new Date(),
    }));
  };

  // Handle Enter key press in input
  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  // Handle Escape key to close chat
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && session.isOpen) {
        closeChat();
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => {
      document.removeEventListener('keydown', handleKeyDown);
    };
  }, [session.isOpen]);

  // Handle edge cases and error handling
  const handleInputFocus = () => {
    // Ensure input is properly focused
    if (session.isOpen) {
      const inputElement = document.querySelector('.chatbot-input') as HTMLTextAreaElement;
      if (inputElement && document.activeElement !== inputElement) {
        inputElement.focus();
      }
    }
  };

  // Cleanup function for React lifecycle
  useEffect(() => {
    return () => {
      // Any cleanup needed when component unmounts
    };
  }, []);

  return (
    <div className="floating-chatbot">
      {/* Floating button - visible when chat is closed */}
      {!session.isOpen && (
        <div
          className="chatbot-button"
          style={{
            position: 'fixed',
            bottom: mergedConfig.position.bottom,
            right: mergedConfig.position.right,
            zIndex: 1000,
          }}
          onClick={toggleChat}
          aria-label="Open chatbot"
          role="button"
        >
          <div className="chatbot-icon" aria-hidden="true">💬</div>
        </div>
      )}

      {/* Chat container - visible when chat is open */}
      {session.isOpen && (
        <div
          className="chatbot-container"
          style={{
            position: 'fixed',
            bottom: mergedConfig.position.bottom,
            right: mergedConfig.position.right,
            zIndex: 1000,
          }}
          onKeyDown={(e) => {
            if (e.key === 'Escape') {
              closeChat();
            }
          }}
          tabIndex={0}
        >
          {/* Chat header */}
          <div
            className="chatbot-header"
            style={{ backgroundColor: mergedConfig.headerColor }}
          >
            <div className="chatbot-header-title">{mergedConfig.botName}</div>
            <button
              className="chatbot-close-button"
              onClick={closeChat}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>

          {/* Messages container */}
          <div className="chatbot-messages" role="log" aria-live="polite">
            {session.messages.map((message) => (
              <div
                key={message.id}
                className={`chatbot-message ${message.sender}-message`}
                role="listitem"
              >
                <div className="message-content">{message.content}</div>
                <div className="message-time" aria-label={`Sent at ${message.timestamp.toLocaleTimeString()}`}>
                  {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                </div>
              </div>
            ))}

            {session.isTyping && (
              <div className="chatbot-message bot-message typing-indicator" role="status" aria-label="Bot is typing">
                <div className="typing-dots">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} aria-hidden="true" />
          </div>

          {/* Input area */}
          <div className="chatbot-input-area">
            <textarea
              className="chatbot-input"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Type your message..."
              rows={1}
              aria-label="Type your message"
              onFocus={handleInputFocus}
            />
            <button
              className="chatbot-send-button"
              onClick={handleSendMessage}
              disabled={!validateMessage(inputValue.trim())}
              aria-label="Send message"
            >
              Send
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default Chatbot;