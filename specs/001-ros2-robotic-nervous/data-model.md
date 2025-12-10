# Data Model: Physical AI Book — Module 1: The Robotic Nervous System (ROS 2)

## Core Entities

### Book Module
- **Name**: Module identifier (e.g., "module-1-ros2")
- **Title**: Display title ("The Robotic Nervous System (ROS 2)")
- **Description**: Brief description of the module content
- **Chapters**: Array of chapter references
- **WordCount**: Estimated word count for the module
- **Status**: Draft/Review/Complete

### Chapter
- **ID**: Unique identifier for the chapter
- **Title**: Chapter title
- **Content**: Markdown content of the chapter
- **ModuleID**: Reference to parent module
- **Order**: Position in the module sequence
- **LearningObjectives**: Array of learning objectives
- **CodeExamples**: Array of code example references
- **Exercises**: Array of exercise references

### Code Example
- **ID**: Unique identifier for the example
- **Title**: Descriptive title
- **Language**: Programming language or format (Python, URDF, etc.)
- **Code**: The actual code content
- **Description**: Explanation of what the example demonstrates
- **FilePath**: Path to the example file in the examples directory
- **ValidationStatus**: Validated/NeedsReview/Invalid
- **ROSVersion**: Compatible ROS 2 version

### URDF Model
- **ID**: Unique identifier for the URDF model
- **Name**: Model name (e.g., "simple_humanoid")
- **Content**: URDF XML content
- **Description**: Description of the robot model
- **Links**: Array of link definitions
- **Joints**: Array of joint definitions
- **Sensors**: Array of sensor definitions
- **Actuators**: Array of actuator definitions
- **ValidationStatus**: Validated/NeedsReview/Invalid

### Chat Session
- **ID**: Unique session identifier
- **UserID**: User identifier (for tracking purposes)
- **Messages**: Array of message objects
- **Timestamp**: Creation time
- **ModuleID**: Reference to the module being discussed
- **SourceChunks**: Retrieved content chunks that informed responses

### Message
- **ID**: Unique message identifier
- **SessionID**: Reference to parent session
- **Role**: "user" or "assistant"
- **Content**: The message content
- **Timestamp**: When the message was created
- **SourceReferences**: Array of book content references used in response

### Retrieved Chunk
- **ID**: Unique identifier for the retrieved chunk
- **Content**: The text chunk retrieved from book
- **SourceModule**: Module where chunk originated
- **SourceChapter**: Chapter where chunk originated
- **SourceSection**: Section within the chapter
- **Embedding**: Vector representation for similarity search
- **ChunkType**: Paragraph/section/semantic-unit

## Relationships

### Module → Chapter
- One-to-Many relationship
- Module contains multiple chapters in a specific order

### Chapter → Code Example
- One-to-Many relationship
- Chapter may reference multiple code examples

### Chapter → URDF Model
- One-to-Many relationship
- Chapter may reference multiple URDF models

### Chat Session → Message
- One-to-Many relationship
- Session contains multiple messages in chronological order

### Message → Retrieved Chunk
- Many-to-Many relationship
- Messages may reference multiple chunks for grounding

## Validation Rules

### Book Module
- Title must be 5-100 characters
- Word count must be between 4,000 and 7,000 for Module 1
- Status must be one of: Draft, Review, Complete

### Chapter
- Title must be 5-100 characters
- Order must be a positive integer
- Content must be valid Markdown
- ModuleID must reference an existing module

### Code Example
- Language must be one of: Python, URDF, Launch, Bash
- ValidationStatus must be one of: Validated, NeedsReview, Invalid
- ROSVersion must be "Humble" or newer

### URDF Model
- Content must be valid XML
- ValidationStatus must be one of: Validated, NeedsReview, Invalid
- Must contain at least one link and one joint

### Chat Session
- ModuleID must reference an existing module
- Messages must be in chronological order

### Message
- Role must be one of: user, assistant
- Content must not be empty
- SessionID must reference an existing session

### Retrieved Chunk
- ChunkType must be one of: paragraph, section, semantic-unit
- SourceModule and SourceChapter must reference existing entities
- Embedding must be a valid vector representation