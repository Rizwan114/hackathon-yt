/**
 * Chat component for interacting with the book-based chatbot
 */
import React, { useState, useEffect, useRef } from 'react';
import chatAPI from '../services/chatAPI';
import sessionService from '../services/session';
import './chat.css'; // Import the CSS file

const ChatComponent = () => {
  const [messages, setMessages] = useState([]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  // Initialize session on component mount
  useEffect(() => {
    // Try to load existing session from storage
    const existingSession = sessionService.loadSessionFromStorage();
    if (!existingSession) {
      // Create a new session if none exists
      const userId = `user_${Date.now()}`;
      sessionService.initializeSession(userId);
    }

    scrollToBottom();
  }, []);

  // Scroll to bottom of messages
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = async () => {
    if (!inputMessage.trim()) return;

    // Get session info
    const sessionId = sessionService.getCurrentSessionId();
    const userId = sessionService.getCurrentUserId();

    // Add user message to the chat
    const userMessage = {
      id: Date.now(),
      text: inputMessage,
      sender: 'user',
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    const currentInput = inputMessage;
    setInputMessage('');

    setIsLoading(true);

    try {
      // Send message to backend using the API service
      const response = await chatAPI.sendMessage(currentInput, {
        user_id: userId,
        session_id: sessionId,
        module_id: 'module-1-ros2',
        top_k: 5,
        min_similarity: 0.3
      });

      // Add bot response to the chat
      const botMessage = {
        id: Date.now() + 1,
        text: response.response,
        sender: 'bot',
        sources: response.sources,
        timestamp: response.timestamp
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      // Add error message to the chat
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, there was an error processing your request. Please try again.',
        sender: 'system',
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <div className="chat-container">
      <div className="chat-header">
        <h3>Book-Based AI Assistant</h3>
        <p>Ask questions about the Physical AI & Humanoid Robotics textbook</p>
      </div>

      <div className="chat-messages">
        {messages.length === 0 ? (
          <div className="welcome-message">
            <p>Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics textbook.</p>
            <p>Ask me any questions about the book content and I'll provide answers based on the material.</p>
          </div>
        ) : (
          messages.map((message) => (
            <div
              key={message.id}
              className={`message ${message.sender}-message`}
            >
              <div className="message-content">
                <p>{message.text}</p>
                {message.sources && message.sources.length > 0 && (
                  <div className="sources">
                    <details>
                      <summary>Sources</summary>
                      <ul>
                        {message.sources.map((source, index) => (
                          <li key={index}>
                            <strong>{source.chapter} - {source.section}:</strong> {source.content.substring(0, 100)}...
                          </li>
                        ))}
                      </ul>
                    </details>
                  </div>
                )}
              </div>
              <div className="message-timestamp">
                {new Date(message.timestamp).toLocaleTimeString()}
              </div>
            </div>
          ))
        )}
        {isLoading && (
          <div className="message bot-message">
            <div className="message-content">
              <p>Thinking...</p>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <div className="chat-input-area">
        <textarea
          value={inputMessage}
          onChange={(e) => setInputMessage(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Ask a question about the book..."
          rows="3"
          disabled={isLoading}
        />
        <button
          onClick={handleSendMessage}
          disabled={isLoading || !inputMessage.trim()}
          className="send-button"
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>
    </div>
  );
};

export default ChatComponent;