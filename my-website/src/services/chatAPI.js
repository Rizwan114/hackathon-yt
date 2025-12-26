/**
 * API service for chat functionality
 */
class ChatAPI {
  constructor(baseURL = 'http://localhost:8000') {
    this.baseURL = baseURL;
  }

  /**
   * Send a message to the chat endpoint
   */
  async sendMessage(message, options = {}) {
    const {
      user_id = 'frontend-user',
      session_id = null,
      module_id = 'module-1-ros2',
      top_k = 5,
      min_similarity = 0.3
    } = options;

    try {
      const response = await fetch(`${this.baseURL}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message,
          user_id,
          session_id,
          module_id,
          top_k,
          min_similarity
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error sending message:', error);
      throw error;
    }
  }

  /**
   * Create a new session
   */
  async createSession(user_id, initial_query = null) {
    try {
      const response = await fetch(`${this.baseURL}/sessions`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          user_id,
          initial_query
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error creating session:', error);
      throw error;
    }
  }

  /**
   * Add a message to an existing session
   */
  async addMessageToSession(session_id, message, options = {}) {
    const {
      user_id = 'frontend-user',
      module_id = 'module-1-ros2',
      top_k = 5,
      min_similarity = 0.3
    } = options;

    try {
      const response = await fetch(`${this.baseURL}/sessions/${session_id}/messages`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message,
          user_id,
          module_id,
          top_k,
          min_similarity
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error adding message to session:', error);
      throw error;
    }
  }

  /**
   * Validate content against book
   */
  async validateContent(text, module_id = 'module-1-ros2') {
    try {
      const response = await fetch(`${this.baseURL}/validate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          text,
          module_id
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error validating content:', error);
      throw error;
    }
  }

  /**
   * Retrieve relevant content based on query
   */
  async retrieveContent(query, options = {}) {
    const {
      module_id = 'module-1-ros2',
      top_k = 5,
      min_similarity = 0.3
    } = options;

    try {
      const response = await fetch(`${this.baseURL}/retrieve`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query,
          module_id,
          top_k,
          min_similarity
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error retrieving content:', error);
      throw error;
    }
  }

  /**
   * Check API health
   */
  async healthCheck() {
    try {
      const response = await fetch(`${this.baseURL}/health`);

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error checking health:', error);
      throw error;
    }
  }
}

// Create a singleton instance
const chatAPI = new ChatAPI();

export default chatAPI;