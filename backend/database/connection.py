"""
Database connection utilities for chat session management
"""
import os
import asyncpg
from typing import Optional
import logging
from contextlib import asynccontextmanager

logger = logging.getLogger(__name__)

class DatabaseConnection:
    """
    Database connection manager for chat session management
    """

    def __init__(self):
        """
        Initialize the database connection manager
        """
        self.pool: Optional[asyncpg.Pool] = None
        self.database_url = os.getenv("NEON_DATABASE_URL", "")

        if not self.database_url:
            logger.warning("NEON_DATABASE_URL environment variable not set")

    async def connect(self):
        """
        Establish connection to the database
        """
        if not self.database_url:
            logger.error("Cannot connect: NEON_DATABASE_URL not set")
            return

        try:
            self.pool = await asyncpg.create_pool(
                self.database_url,
                min_size=1,
                max_size=10,
                command_timeout=60
            )
            logger.info("Database connection pool created successfully")
        except Exception as e:
            logger.error(f"Failed to create database connection pool: {e}")
            raise

    async def disconnect(self):
        """
        Close the database connection
        """
        if self.pool:
            await self.pool.close()
            logger.info("Database connection pool closed")

    @asynccontextmanager
    async def get_connection(self):
        """
        Get a database connection from the pool
        """
        if not self.pool:
            raise RuntimeError("Database not connected")

        conn = await self.pool.acquire()
        try:
            yield conn
        finally:
            await self.pool.release(conn)

# Global database instance
db = DatabaseConnection()