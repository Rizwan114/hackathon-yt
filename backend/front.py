from agents import Agent, Runner, OpenAIChatCompletionsModel, AsyncOpenAI
from agents import set_tracing_disabled, function_tool
import os
from dotenv import load_dotenv
from agents import enable_verbose_stdout_logging
import logging

# Set up basic error handling
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# enable_verbose_stdout_logging()

load_dotenv()
set_tracing_disabled(disabled=True)

gemini_api_key = os.getenv("OPENROUTER_API_KEY")
if not gemini_api_key:
    logger.error("GEMINI_API_KEY not found in environment variables")
    exit(1)

provider = AsyncOpenAI(
    api_key=gemini_api_key,
    base_url="https://openrouter.ai/api/v1"
)

model = OpenAIChatCompletionsModel(
    model="xiaomi/mimo-v2-flash:free",
    openai_client=provider
)

import cohere
from qdrant_client import QdrantClient

# Initialize Cohere client
cohere_client = cohere.Client("Weyiywa7HxvabgJtc96onPL7rLTvQGsdmvLzYaCC")
# Connect to Qdrant
qdrant = QdrantClient(
    url="https://bf55218d-3692-4c0f-93a1-d46514492ea9.us-east4-0.gcp.cloud.qdrant.io:6333",
    api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.OdZwxQuabpvo2Eufde55QUULYP1-u117Nj53aoS1JAg",
)


def get_embedding(text):
    """Get embedding vector from Cohere Embed v3"""
    try:
        response = cohere_client.embed(
            model="embed-english-v3.0",
            input_type="search_query",  # Use search_query for queries
            texts=[text],
        )
        return response.embeddings[0]  # Return the first embedding
    except Exception as e:
        logger.error(f"Error getting embedding: {e}")
        raise


@function_tool
def retrieve(query):
    try:
        embedding = get_embedding(query)
        result = qdrant.query_points(
            collection_name="humanoid_ai_book",
            query=embedding,
            limit=5
        )
        return [point.payload["text"] for point in result.points]
    except Exception as e:
        logger.error(f"Error in retrieve function: {e}")
        return ["Error retrieving information from the book. Please try again later."]


agent = Agent(
    name="Assistant",
    instructions="""
You are an AI tutor for the Physical AI & Humanoid Robotics textbook.
To answer the user question, first call the tool `retrieve` with the user query.
Use ONLY the returned content from `retrieve` to answer.
If the answer is not in the retrieved content, say "I don't know".
""",
    model=model,
    tools=[retrieve]
)

try:
    result = Runner.run_sync(
        agent,
        input="what is  Docusaurus?",
    )
    print(result.final_output)
except Exception as e:
    logger.error(f"Error running agent: {e}")
    print(f"Error: {e}")
    # Provide a graceful fallback
    print("The system is currently unavailable. Please check your API keys and network connection.")