/**
 * Session management service for frontend chat
 */
class SessionService {
  constructor() {
    this.currentSessionId = null;
    this.currentUserId = null;
  }

  /**
   * Initialize a new session
   */
  async initializeSession(userId, initialQuery = null) {
    try {
      // In a real implementation, we would call the backend to create a session
      // For now, we'll simulate session creation
      this.currentUserId = userId;
      this.currentSessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;

      // If there's an initial query, process it
      if (initialQuery) {
        // In a real app, we would send this to the backend
        console.log(`Initial query for session ${this.currentSessionId}:`, initialQuery);
      }

      return {
        session_id: this.currentSessionId,
        user_id: this.currentUserId,
        created_at: new Date().toISOString(),
        message_count: initialQuery ? 1 : 0
      };
    } catch (error) {
      console.error('Error initializing session:', error);
      throw error;
    }
  }

  /**
   * Get the current session ID
   */
  getCurrentSessionId() {
    return this.currentSessionId;
  }

  /**
   * Get the current user ID
   */
  getCurrentUserId() {
    return this.currentUserId;
  }

  /**
   * Check if there's an active session
   */
  hasActiveSession() {
    return !!this.currentSessionId;
  }

  /**
   * End the current session
   */
  endSession() {
    this.currentSessionId = null;
    this.currentUserId = null;
  }

  /**
   * Save session to local storage
   */
  saveSessionToStorage() {
    if (this.currentSessionId) {
      const sessionData = {
        session_id: this.currentSessionId,
        user_id: this.currentUserId,
        timestamp: new Date().toISOString()
      };
      localStorage.setItem('chatSession', JSON.stringify(sessionData));
    }
  }

  /**
   * Load session from local storage
   */
  loadSessionFromStorage() {
    try {
      const sessionData = JSON.parse(localStorage.getItem('chatSession'));
      if (sessionData) {
        // Check if session is still valid (less than 24 hours old)
        const sessionTime = new Date(sessionData.timestamp).getTime();
        const now = new Date().getTime();
        const hourDifference = (now - sessionTime) / (1000 * 60 * 60);

        if (hourDifference < 24) {
          this.currentSessionId = sessionData.session_id;
          this.currentUserId = sessionData.user_id;
          return {
            session_id: this.currentSessionId,
            user_id: this.currentUserId
          };
        } else {
          // Session expired, remove it
          this.clearSessionStorage();
        }
      }
      return null;
    } catch (error) {
      console.error('Error loading session from storage:', error);
      this.clearSessionStorage();
      return null;
    }
  }

  /**
   * Clear session from local storage
   */
  clearSessionStorage() {
    localStorage.removeItem('chatSession');
  }

  /**
   * Resume a session by ID
   */
  async resumeSession(sessionId, userId) {
    this.currentSessionId = sessionId;
    this.currentUserId = userId;

    // Save to storage
    this.saveSessionToStorage();

    return {
      session_id: this.currentSessionId,
      user_id: this.currentUserId
    };
  }
}

// Create a singleton instance
const sessionService = new SessionService();

export default sessionService;