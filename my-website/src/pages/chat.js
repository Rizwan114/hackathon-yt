/**
 * Chat page for the book-based chatbot
 */
import React from 'react';
import ChatComponent from '../components/ChatComponent';
import '../components/chat.css'; // Import chat styles

const ChatPage = () => {
  return (
    <div className="chat-page">
      <header className="chat-page-header">
        <h1>Book-Based AI Assistant</h1>
        <p>Ask questions about the Physical AI & Humanoid Robotics textbook</p>
      </header>

      <main className="chat-page-main">
        <ChatComponent />
      </main>

      <footer className="chat-page-footer">
        <p>This AI assistant only responds based on content from the Physical AI & Humanoid Robotics textbook.</p>
      </footer>
    </div>
  );
};

export default ChatPage;