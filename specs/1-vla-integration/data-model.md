# Data Model: Vision-Language-Action (VLA) Integration

## Entities

### VoiceCommand
- **id**: string (unique identifier)
- **text**: string (transcribed text from speech)
- **timestamp**: datetime (when command was received)
- **confidence**: float (confidence score from speech recognition)
- **user_id**: string (optional, to track command source)
- **status**: enum ['pending', 'processing', 'completed', 'failed']

### TextTranscript
- **id**: string (unique identifier)
- **original_audio_path**: string (path to original audio file)
- **transcript**: string (the transcribed text)
- **language**: string (detected language)
- **duration**: float (audio duration in seconds)
- **processing_metadata**: object (metadata from speech recognition)

### ActionPlan
- **id**: string (unique identifier)
- **command_id**: string (reference to the original voice command)
- **steps**: array (sequence of actions to execute)
- **status**: enum ['planned', 'executing', 'completed', 'failed', 'cancelled']
- **created_at**: datetime
- **updated_at**: datetime
- **execution_context**: object (environmental constraints and conditions)

### ActionStep
- **id**: string (unique identifier)
- **plan_id**: string (reference to parent action plan)
- **step_number**: integer (position in sequence)
- **action_type**: string (e.g., 'navigation', 'manipulation', 'perception')
- **parameters**: object (specific parameters for the action)
- **dependencies**: array (other steps this step depends on)
- **timeout**: integer (seconds before considering failed)
- **retry_count**: integer (number of retries attempted)

### RobotState
- **id**: string (unique identifier)
- **timestamp**: datetime
- **position**: object (x, y, z coordinates)
- **orientation**: object (quaternion or euler angles)
- **joint_states**: object (current joint positions and velocities)
- **battery_level**: float (0-100 percentage)
- **active_sensors**: object (status of active sensors)
- **current_task**: string (currently executing task ID)

### ExecutionContext
- **id**: string (unique identifier)
- **environment**: string (simulation or real environment)
- **obstacles**: array (detected obstacles in environment)
- **constraints**: object (movement or action constraints)
- **feedback**: object (sensor feedback during execution)
- **timestamp**: datetime

## Relationships

- `VoiceCommand` 1 → * `ActionPlan`: One command can generate one or more action plans (in case of complex commands)
- `ActionPlan` 1 → * `ActionStep`: One plan contains multiple steps
- `ActionPlan` * → 1 `RobotState`: Action plans reference current robot state
- `ActionPlan` 1 → 1 `ExecutionContext`: Each plan has execution context
- `ActionStep` * → 1 `RobotState`: Steps may reference robot state during execution

## Validation Rules

1. **VoiceCommand**:
   - `text` must not be empty
   - `confidence` must be between 0 and 1
   - `status` must be one of the defined enum values

2. **ActionPlan**:
   - `steps` array must not be empty
   - `status` must be one of the defined enum values
   - `created_at` must be before `updated_at` if both present

3. **ActionStep**:
   - `step_number` must be non-negative
   - `timeout` must be positive
   - `retry_count` must not exceed maximum allowed retries (e.g., 3)

4. **RobotState**:
   - `position` must have valid coordinates
   - `battery_level` must be between 0 and 100