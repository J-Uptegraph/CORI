# Technical Roadmap - Python & CORI Integration

Achieve a comprehensive understanding and mastery of data structures and algorithms, enabling clear articulation, robust implementation, and practical application in a robotics context, specifically for a humanoid robot like CORI. Each CORI feature challenge is designed to be implemented from scratch in 45 minutes, utilizing the day's core data structure or algorithm while iteratively building on the project's foundation to create innovative and impactful features.

## Schedule
- **5:00–5:30 AM**: Wake up and prepare for the day
- **5:30–7:15 AM**: Research and review the day’s topic & solve easy LeetCode problems
- **6:45–7:15 AM**: Breakfast and study
- **7:30–8:00 AM**: Morning verbal review with ChatGPT

- **5:00–5:45 PM**: Evening verbal review with ChatGPT
- **5:45–7:00 PM**: Solve medium LeetCode problems
- **7:00–7:45 PM**: 45-minute CoderPad-style CORI challenge (customized for the day’s topic)
- **8:00–9:00 PM**: Physical training
- **9:00–10:00 PM**: Shower, relax, dinner, and rest

## Topics Covered
| Topic | Explanation | Robotics Application (CORI) |
|-------|-------------|----------------------------|
| **Arrays/Lists** | Dynamic and fixed-size arrays for efficient data storage and manipulation. Supports O(1) access and O(n) insertion/deletion. | Optimize CORI’s movement history by storing head positions, removing duplicates, and managing memory with a circular buffer. |
| **Strings** | Techniques for manipulation, searching, and pattern matching (e.g., regex). Handles immutable strings and operations like split/join. | Parse natural language commands for laundry sorting, extracting actions and attributes like color or clothing type. |
| **Linked Lists** | Singly, doubly, or circular lists for flexible data organization. Enables O(1) insertion at known positions but O(n) access. | Manage CORI’s task queue with priority-based insertion and deletion for efficient task scheduling. |
| **Stacks & Queues** | LIFO (stack) and FIFO (queue) structures for ordered processing. Used in function calls, BFS/DFS, and undo operations. | Implement state management for CORI, supporting undo operations and task scheduling with deque-based queues. |
| **Hash Maps/Tables** | Key-value storage (Python dictionaries) with O(1) average-case operations. Handles collisions via chaining or probing. | Create an adaptive learning database for CORI to track user preferences and confidence scores with fast lookups. |
| **Trees** | Binary Trees and BSTs with traversals (in-order, pre-order, post-order, BFS, DFS). Used for hierarchical data and decision-making. | Build a decision tree for CORI to classify clothing based on attributes like color and fabric, supporting user corrections. |
| **Graphs** | Adjacency lists/matrices and traversals (BFS, DFS) for interconnected systems. Supports pathfinding and network analysis. | Enable CORI to navigate a household graph, optimizing paths for laundry collection while avoiding obstacles. |
| **Heaps (Priority Queues)** | Min/max-heaps for efficient priority management. O(log n) insert/delete, O(1) peek. Used in scheduling and shortest-path algorithms. | Dynamically prioritize CORI’s tasks based on urgency, energy levels, and user preferences using a heap-based scheduler. |
| **Sorting** | Algorithms like Quick Sort (O(n log n)), Merge Sort (stable), and Insertion Sort (adaptive). Vary in time/space complexity. | Sort laundry items by multiple criteria (color, fabric, priority) to optimize washing efficiency. |
| **Searching** | Binary Search for O(log n) retrieval in sorted data. Variants include finding first/last occurrence or insertion points. | Locate objects in CORI’s sorted spatial database by position, confidence, or time, optimizing search efficiency. |
| **Recursion & Backtracking** | Recursive solutions break problems into subproblems; backtracking explores all possibilities systematically. | Plan CORI’s paths through a house with obstacle avoidance, using recursive backtracking for optimal laundry collection routes. |
| **Dynamic Programming** | Solves complex problems by caching solutions to overlapping subproblems. Uses memoization or tabulation for optimization. | Optimize CORI’s task scheduling and energy usage, maximizing rewards within constraints like battery life and deadlines. |
| **Greedy Algorithms** | Makes locally optimal choices for global optima. Suitable for problems like interval scheduling and resource allocation. | Schedule CORI’s real-time tasks by prioritizing high-efficiency or urgent tasks to maximize productivity. |
| **Two Pointers** | Efficient technique for array/list traversal, often used for finding pairs or optimizing iterations. | Process CORI’s sensor data streams to detect patterns or anomalies in real-time movement or object detection. |
| **Sliding Window** | Optimizes subarray/substring problems by maintaining a dynamic window, reducing time complexity. | Analyze continuous sensor data for CORI to detect patterns in laundry item sequences or spatial movements. |

## 1. Arrays/Lists

### Key Concepts:
- **Dynamic Arrays (Python Lists)**: Resizable arrays with O(1) amortized append, O(n) insertion/deletion
- **Fixed-size Arrays**: Using `array.array()` or NumPy arrays for memory efficiency
- **List Comprehensions**: Pythonic way to create and transform lists
- **Slicing**: Efficient array manipulation with `arr[start:end:step]`
- **Common Operations**: append, insert, remove, pop, index, reverse, sort
- **Time Complexities**: Access O(1), Search O(n), Insertion O(n), Deletion O(n)
- **Space Complexity**: O(n) for storage

### Notes:
-

### LeetCode Problems (AM):
_[Track morning problem-solving sessions]_

### ChatGPT Conversation (AM):
_[Record key discussions and clarifications from morning study]_

### ChatGPT Conversation (PM):
_[Evening review and deeper concept exploration]_

### LeetCode Problems (PM):
_[Evening practice problems and solutions]_

### CORI Specific Challenge:
**Problem: Robot Path Memory Optimization**

You're implementing CORI's movement history system. The robot stores its head positions as a list of angles (in radians) where each position represents where CORI looked during laundry sorting.

```python
def optimize_movement_history(positions, max_memory=100):
    """
    Given a list of head positions, implement a system that:
    1. Keeps only the most recent max_memory positions
    2. Removes consecutive duplicate positions (robot staying still)
    3. Returns the optimized position list and the compression ratio
    
    Args:
        positions: List[float] - head angles in radians
        max_memory: int - maximum positions to keep
    
    Returns:
        Tuple[List[float], float] - (optimized_positions, compression_ratio)
    
    Example:
        positions = [0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 0.0]
        max_memory = 6
        # Should return ([0.0, 0.5, 1.0, 0.0], 0.5)
    """
    pass

# Additional challenge: Implement a circular buffer for real-time position tracking
class HeadPositionBuffer:
    def __init__(self, capacity):
        pass
    
    def add_position(self, angle):
        pass
    
    def get_recent_positions(self, count):
        pass
```

**Time Limit: 30 minutes**
**Follow-up Questions**: How would you handle concurrent access? What if positions need timestamps?

---

## 2. Strings

### Key Concepts:
- **Immutability**: Python strings are immutable; operations create new strings
- **String Methods**: split(), join(), strip(), replace(), find(), startswith(), endswith()
- **String Formatting**: f-strings, .format(), % formatting
- **Pattern Matching**: Regular expressions with `re` module
- **Encoding/Decoding**: UTF-8, ASCII, Unicode handling
- **Common Patterns**: Palindromes, anagrams, subsequences, substrings
- **Time Complexities**: Most operations O(n), concatenation can be O(n²) without optimization

### Notes:
-

### LeetCode Problems (AM):
_[Track morning problem-solving sessions]_

### ChatGPT Conversation (AM):
_[Record key discussions and clarifications from morning study]_

### ChatGPT Conversation (PM):
_[Evening review and deeper concept exploration]_

### LeetCode Problems (PM):
_[Evening practice problems and solutions]_

### CORI Specific Challenge:
**Problem: Natural Language Command Parser**

CORI needs to parse user commands for laundry sorting. Implement a command parser that extracts color and action from natural language inputs.

```python
def parse_laundry_command(command):
    """
    Parse natural language commands for CORI's laundry system.
    
    Valid patterns:
    - "sort [color] items" -> ("sort", color)
    - "find my [color] [clothing_type]" -> ("find", color, clothing_type)  
    - "where did you put the [color] clothes" -> ("locate", color)
    - "show me [color] statistics" -> ("stats", color)
    
    Args:
        command: str - user's natural language command
        
    Returns:
        Dict with parsed components or error message
        
    Examples:
        "sort red items" -> {"action": "sort", "color": "red"}
        "find my blue shirt" -> {"action": "find", "color": "blue", "item": "shirt"}
        "invalid command" -> {"error": "Command not recognized"}
    """
    pass

def validate_color(color_name):
    """
    Check if the color name is in CORI's known color database.
    Support variations like "dark blue", "light green", etc.
    """
    known_colors = ["red", "blue", "green", "yellow", "black", "white", "purple", "orange"]
    pass

def extract_clothing_type(text):
    """
    Extract clothing type from text (shirt, pants, socks, etc.)
    Handle plurals and common variations.
    """
    pass
```

**Time Limit: 35 minutes**
**Follow-up**: How would you handle typos and fuzzy matching?

---

## 3. Linked Lists

### Key Concepts:
- **Singly Linked Lists**: Each node points to the next node
- **Doubly Linked Lists**: Nodes have pointers to both next and previous
- **Circular Linked Lists**: Last node points back to the first
- **Common Operations**: Insert, delete, search, reverse, merge
- **Two Pointer Technique**: Fast/slow pointers for cycle detection, finding middle
- **Time Complexities**: Access O(n), Search O(n), Insertion O(1) at known position
- **Memory**: More overhead than arrays due to pointer storage

### Notes:
-

### LeetCode Problems (AM):
_[Track morning problem-solving sessions]_

### ChatGPT Conversation (AM):
_[Record key discussions and clarifications from morning study]_

### ChatGPT Conversation (PM):
_[Evening review and deeper concept exploration]_

### LeetCode Problems (PM):
_[Evening practice problems and solutions]_

### CORI Specific Challenge:
**Problem: Robot Task Queue with Priority**

Implement a doubly-linked list to manage CORI's task queue where each task has a priority and can be inserted, removed, or reordered efficiently.

```python
class Task:
    def __init__(self, task_id, description, priority, estimated_time):
        self.task_id = task_id
        self.description = description  # "sort_red_items", "find_blue_shirt", etc.
        self.priority = priority        # 1 (highest) to 10 (lowest)
        self.estimated_time = estimated_time  # seconds
        self.next = None
        self.prev = None

class TaskQueue:
    def __init__(self):
        self.head = None
        self.tail = None
        self.size = 0
    
    def add_task(self, task):
        """Add task maintaining priority order (highest priority first)"""
        pass
    
    def get_next_task(self):
        """Remove and return the highest priority task"""
        pass
    
    def cancel_task(self, task_id):
        """Remove specific task by ID"""
        pass
    
    def reorder_by_time(self):
        """Reorder tasks to minimize total completion time"""
        pass
    
    def display_queue(self):
        """Return list of tasks in current order"""
        pass

# Test case
def test_cori_task_queue():
    queue = TaskQueue()
    
    # Add various laundry tasks
    tasks = [
        Task(1, "sort_red_items", 3, 30),
        Task(2, "find_blue_shirt", 1, 15),  # Urgent - user asking
        Task(3, "analyze_hamper", 5, 60),
        Task(4, "sort_white_items", 2, 25)
    ]
    
    for task in tasks:
        queue.add_task(task)
    
    # Should execute in priority order: blue_shirt -> white_items -> red_items -> analyze_hamper
```

**Time Limit: 40 minutes**
**Follow-up**: How would you handle task dependencies?

---

## 4. Stacks & Queues

### Key Concepts:
- **Stack (LIFO)**: Last In, First Out - push(), pop(), peek(), isEmpty()
- **Queue (FIFO)**: First In, First Out - enqueue(), dequeue(), front(), isEmpty()
- **Implementation**: Using lists, collections.deque, or linked lists
- **Applications**: Function calls, expression evaluation, BFS/DFS, undo operations
- **Time Complexities**: All operations O(1) with proper implementation
- **deque**: Double-ended queue, efficient operations at both ends

### Notes:
-

### LeetCode Problems (AM):
_[Track morning problem-solving sessions]_

### ChatGPT Conversation (AM):
_[Record key discussions and clarifications from morning study]_

### ChatGPT Conversation (PM):
_[Evening review and deeper concept exploration]_

### LeetCode Problems (PM):
_[Evening practice problems and solutions]_

### CORI Specific Challenge:
**Problem: Robot State Management System**

Implement a state management system for CORI using stacks and queues to handle undo operations and task scheduling.

```python
from collections import deque
from enum import Enum

class RobotAction(Enum):
    MOVE_HEAD = "move_head"
    SORT_ITEM = "sort_item"
    UPDATE_DATABASE = "update_database"
    NAVIGATE = "navigate"

class ActionState:
    def __init__(self, action_type, previous_state, new_state, timestamp):
        self.action_type = action_type
        self.previous_state = previous_state  # State before action
        self.new_state = new_state           # State after action
        self.timestamp = timestamp

class CORIStateManager:
    def __init__(self, max_undo_history=50):
        self.undo_stack = []  # Stack for undo operations
        self.task_queue = deque()  # Queue for pending tasks
        self.max_undo_history = max_undo_history
        self.current_state = {}
    
    def execute_action(self, action_type, new_state_data):
        """
        Execute an action and save state for potential undo.
        Manage undo history size.
        """
        pass
    
    def undo_last_action(self):
        """
        Undo the most recent action and restore previous state.
        Return success/failure and action description.
        """
        pass
    
    def schedule_task(self, task_description, priority=5):
        """
        Add task to queue. Higher priority tasks go to front.
        """
        pass
    
    def process_next_task(self):
        """
        Process the next task in queue and execute corresponding action.
        """
        pass
    
    def get_undo_history(self, count=10):
        """
        Return the last 'count' actions that can be undone.
        """
        pass

# Example usage and test
def test_state_manager():
    manager = CORIStateManager()
    
    # Simulate CORI operations
    manager.execute_action(RobotAction.MOVE_HEAD, {"head_angle": 0.5})
    manager.execute_action(RobotAction.SORT_ITEM, {"item": "red_shirt", "location": "colors_bin"})
    manager.execute_action(RobotAction.UPDATE_DATABASE, {"color": "red", "category": "colors"})
    
    # Schedule future tasks
    manager.schedule_task("sort_blue_items", priority=3)
    manager.schedule_task("find_missing_sock", priority=1)  # Urgent
    
    # Test undo functionality
    print(manager.undo_last_action())  # Should undo database update
    print(manager.undo_last_action())  # Should undo item sorting
    
    # Process queued tasks
    manager.process_next_task()  # Should handle missing sock first
```

**Time Limit: 35 minutes**
**Follow-up**: How would you persist the undo history across robot restarts?

---

## 5. Hash Maps/Tables (Dictionaries)

### Key Concepts:
- **Hash Function**: Maps keys to indices, should minimize collisions
- **Collision Handling**: Chaining (lists) vs Open Addressing (probing)
- **Load Factor**: Ratio of elements to buckets, affects performance
- **Python Dictionaries**: Hash tables with guaranteed insertion order (Python 3.7+)
- **Time Complexities**: Average O(1) for get/set/delete, worst case O(n)
- **Sets**: Hash tables without values, for membership testing
- **defaultdict & Counter**: Specialized dictionary subclasses

### Notes:
-

### LeetCode Problems (AM):
_[Track morning problem-solving sessions]_

### ChatGPT Conversation (AM):
_[Record key discussions and clarifications from morning study]_

### ChatGPT Conversation (PM):
_[Evening review and deeper concept exploration]_

### LeetCode Problems (PM):
_[Evening practice problems and solutions]_

### CORI Specific Challenge:
**Problem: CORI's Adaptive Learning Database**

Implement an intelligent caching system for CORI's learning database that tracks user preferences, confidence scores, and pattern recognition with efficient lookups and updates.

```python
from collections import defaultdict, Counter
from typing import Dict, List, Tuple
import time

class LearningDatabase:
    def __init__(self, cache_size=1000):
        self.user_preferences = {}  # user_id -> {color: category}
        self.confidence_scores = defaultdict(float)  # (user, color) -> confidence
        self.pattern_cache = {}     # Recently accessed patterns for fast lookup
        self.access_frequency = Counter()  # Track most accessed patterns
        self.cache_size = cache_size
        self.last_access = {}       # For LRU cache implementation
    
    def learn_preference(self, user_id, color, category, confidence_delta=0.1):
        """
        Update user preference and confidence score.
        Implement confidence score updates based on correct predictions.
        """
        pass
    
    def get_prediction(self, user_id, color):
        """
        Get category prediction with confidence score.
        Update access frequency and cache.
        Returns: (predicted_category, confidence_score)
        """
        pass
    
    def find_similar_users(self, user_id, min_similarity=0.7):
        """
        Find users with similar sorting preferences.
        Use Jaccard similarity or cosine similarity.
        Returns: List of (similar_user_id, similarity_score)
        """
        pass
    
    def detect_patterns(self, user_id):
        """
        Analyze user's preferences to detect patterns:
        - Consistent color mappings
        - Unusual categorizations
        - Learning progress over time
        
        Returns: Dict with pattern analysis
        """
        pass
    
    def optimize_cache(self):
        """
        Implement LRU cache eviction for pattern_cache.
        Keep most frequently accessed patterns.
        """
        pass
    
    def get_user_stats(self, user_id):
        """
        Generate comprehensive statistics for a user:
        - Total items sorted
        - Confidence scores by color
        - Learning progress metrics
        - Consistency scores
        """
        pass

class ColorPatternAnalyzer:
    def __init__(self, database: LearningDatabase):
        self.db = database
        self.color_relationships = defaultdict(set)  # Track which colors appear together
    
    def analyze_color_clusters(self, user_id):
        """
        Group colors by how user categorizes them.
        Identify outliers and inconsistencies.
        """
        pass
    
    def suggest_clarifications(self, user_id):
        """
        Suggest questions to ask user based on uncertain patterns.
        Returns prioritized list of (color, question, importance_score)
        """
        pass

# Test the system
def test_learning_system():
    db = LearningDatabase()
    analyzer = ColorPatternAnalyzer(db)
    
    # Simulate user learning over time
    user_data = [
        ("user1", "red", "colors", True),
        ("user1", "blue", "colors", True), 
        ("user1", "black", "darks", True),
        ("user1", "navy", "darks", False),  # User corrected CORI
        ("user1", "navy", "darks", True),   # CORI learned
    ]
    
    for user_id, color, category, was_correct in user_data:
        confidence_change = 0.1 if was_correct else -0.2
        db.learn_preference(user_id, color, category, confidence_change)
    
    # Test predictions and analysis
    prediction = db.get_prediction("user1", "navy")
    patterns = analyzer.analyze_color_clusters("user1")
    suggestions = analyzer.suggest_clarifications("user1")
```

**Time Limit: 45 minutes**
**Follow-up**: How would you handle hash collisions in a distributed system?

---

## 6. Trees

### Key Concepts:
- **Binary Tree**: Each node has at most 2 children (left, right)
- **Binary Search Tree (BST)**: Left subtree < root < right subtree
- **Tree Traversals**: Inorder (LNR), Preorder (NLR), Postorder (LRN)
- **BFS vs DFS**: Breadth-First (level order) vs Depth-First traversals
- **Balanced Trees**: AVL, Red-Black trees for guaranteed O(log n) operations
- **Tree Properties**: Height, depth, complete, full, perfect trees
- **Applications**: File systems, expression parsing, decision trees

### Notes:
-

### LeetCode Problems (AM):
_[Track morning problem-solving sessions]_

### ChatGPT Conversation (AM):
_[Record key discussions and clarifications from morning study]_

### ChatGPT Conversation (PM):
_[Evening review and deeper concept exploration]_

### LeetCode Problems (PM):
_[Evening practice problems and solutions]_

### CORI Specific Challenge:
**Problem: CORI's Decision Tree for Clothing Classification**

Build a decision tree system that helps CORI classify clothing items based on multiple attributes and learn from user corrections.

```python
from enum import Enum
from typing import Optional, List, Dict, Any

class ClothingAttribute(Enum):
    COLOR = "color"
    FABRIC = "fabric" 
    SIZE = "size"
    PATTERN = "pattern"
    CARE_INSTRUCTIONS = "care"

class SortingCategory(Enum):
    LIGHTS = "lights"
    DARKS = "darks" 
    COLORS = "colors"
    DELICATES = "delicates"
    SEPARATE = "separate"

class DecisionNode:
    def __init__(self, attribute: ClothingAttribute, threshold_or_values=None):
        self.attribute = attribute
        self.threshold_or_values = threshold_or_values  # For splitting decisions
        self.children = {}  # attribute_value -> child_node
        self.prediction = None  # Leaf node prediction
        self.confidence = 0.0
        self.samples_count = 0

class ClothingDecisionTree:
    def __init__(self):
        self.root = None
        self.training_data = []  # Store for retraining
        self.user_corrections = []  # Track when user corrects CORI
    
    def add_training_example(self, attributes: Dict[ClothingAttribute, Any], 
                           correct_category: SortingCategory, user_corrected=False):
        """
        Add a training example to the tree.
        If user_corrected=True, weight this example more heavily.
        """
        pass
    
    def build_tree(self, examples: List[Tuple], current_attributes: List[ClothingAttribute]):
        """
        Recursively build decision tree using information gain or Gini impurity.
        
        Returns: DecisionNode representing the (sub)tree
        """
        pass
    
    def calculate_information_gain(self, examples: List, attribute: ClothingAttribute):
        """
        Calculate information gain for splitting on given attribute.
        Use entropy-based calculation.
        """
        pass
    
    def predict(self, clothing_attributes: Dict[ClothingAttribute, Any]):
        """
        Traverse tree to predict sorting category.
        
        Returns: (predicted_category, confidence_score, decision_path)
        """
        pass
    
    def update_with_correction(self, clothing_attributes: Dict, 
                             wrong_prediction: SortingCategory,
                             correct_category: SortingCategory):
        """
        Handle user correction by updating tree or flagging for retraining.
        """
        pass
    
    def get_decision_explanation(self, clothing_attributes: Dict):
        """
        Provide human-readable explanation of why CORI made its decision.
        
        Returns: List of decision steps like:
        ["Color is blue -> check fabric", "Fabric is cotton -> colors bin"]
        """
        pass
    
    def prune_tree(self, validation_data: List):
        """
        Remove overfitted branches that don't improve validation accuracy.
        """
        pass

class CORIClothingClassifier:
    def __init__(self):
        self.decision_tree = ClothingDecisionTree()
        self.attribute_extractors = self._setup_extractors()
        self.uncertainty_threshold = 0.6  # Ask user if confidence below this
    
    def _setup_extractors(self):
        """Setup functions to extract attributes from raw clothing data."""
        return {
            ClothingAttribute.COLOR: self._extract_color,
            ClothingAttribute.FABRIC: self._extract_fabric,
            ClothingAttribute.PATTERN: self._extract_pattern,
        }
    
    def _extract_color(self, clothing_item) -> str:
        """Extract dominant color from clothing item."""
        pass
    
    def _extract_fabric(self, clothing_item) -> str:
        """Classify fabric type (cotton, wool, synthetic, etc.)."""
        pass
    
    def _extract_pattern(self, clothing_item) -> str:
        """Detect patterns (solid, striped, printed, etc.)."""
        pass
    
    def classify_item(self, clothing_item):
        """
        Full classification pipeline:
        1. Extract attributes
        2. Get tree prediction
        3. Decide whether to ask user for confirmation
        """
        pass
    
    def interactive_learning_session(self, clothing_items: List):
        """
        Process multiple items, asking for user input when uncertain.
        Update tree based on user feedback.
        """
        pass

# Test the decision tree system
def test_clothing_classifier():
    classifier = CORIClothingClassifier()
    
    # Training data: (attributes, correct_category)
    training_examples = [
        ({ClothingAttribute.COLOR: "white", ClothingAttribute.FABRIC: "cotton"}, SortingCategory.LIGHTS),
        ({ClothingAttribute.COLOR: "black", ClothingAttribute.FABRIC: "cotton"}, SortingCategory.DARKS),
        ({ClothingAttribute.COLOR: "red", ClothingAttribute.FABRIC: "cotton"}, SortingCategory.COLORS),
        ({ClothingAttribute.COLOR: "white", ClothingAttribute.FABRIC: "silk"}, SortingCategory.DELICATES),
        # Add edge cases and user corrections
    ]
    
    # Build initial tree
    for attributes, category in training_examples:
        classifier.decision_tree.add_training_example(attributes, category)
    
    # Test classification and explanations
    test_item = {ClothingAttribute.COLOR: "navy", ClothingAttribute.FABRIC: "denim"}
    prediction, confidence, path = classifier.decision_tree.predict(test_item)
    explanation = classifier.decision_tree.get_decision_explanation(test_item)
    
    print(f"Prediction: {prediction}, Confidence: {confidence}")
    print(f"Decision path: {explanation}")
```

**Time Limit: 45 minutes**
**Follow-up**: How would you handle continuous learning as new clothing types are introduced?

---

## 7. Graphs

### Key Concepts:
- **Representation**: Adjacency List vs Adjacency Matrix
- **Types**: Directed/Undirected, Weighted/Unweighted, Cyclic/Acyclic
- **Traversals**: BFS (shortest path), DFS (exploring paths)
- **Algorithms**: Dijkstra's (shortest path), Topological Sort, Union-Find
- **Applications**: Social networks, routing, dependency resolution
- **Time Complexities**: BFS/DFS O(V+E), Dijkstra O((V+E)log V)

### Notes:
-

### LeetCode Problems (AM):
_[Track morning problem-solving sessions]_

### ChatGPT Conversation (AM):
_[Record key discussions and clarifications from morning study]_

### ChatGPT Conversation (PM):
_[Evening review and deeper concept exploration]_

### LeetCode Problems (PM):
_[Evening practice problems and solutions]_

### CORI Specific Challenge:
**Problem: CORI's Household Navigation Graph**

Implement a graph-based navigation system for CORI to find optimal paths through a house while considering obstacles, carrying capacity, and energy efficiency.

```python
from collections import defaultdict, deque
import heapq
from typing import List, Tuple, Dict, Set
from enum import Enum

class RoomType(Enum):
    LAUNDRY_ROOM = "laundry_room"
    BEDROOM = "bedroom"
    BATHROOM = "bathroom"
    HALLWAY = "hallway"
    LIVING_ROOM = "living_room"
    KITCHEN = "kitchen"

class EdgeType(Enum):
    DOORWAY = "doorway"
    STAIRS = "stairs" 
    NARROW_PASSAGE = "narrow_passage"
    OPEN_SPACE = "open_space"

class HouseholdGraph:
    def __init__(self):
        self.rooms = {}  # room_id -> RoomType
        self.adjacency_list = defaultdict(list)  # room_id -> [(neighbor_id, edge_type, distance, energy_cost)]
        self.room_contents = defaultdict(set)  # room_id -> set of items
        self.blocked_edges = set()  # Temporarily blocked paths
        self.room_coordinates = {}  # room_id -> (x, y) for visualization
    
    def add_room(self, room_id: str, room_type: RoomType, coordinates: Tuple[float, float]):
        """Add a room to the house graph."""
        pass
    
    def add_connection(self, room1: str, room2: str, edge_type: EdgeType, 
                      distance: float, base_energy_cost: float):
        """
        Add bidirectional connection between rooms.
        Energy cost might vary based on robot's load.
        """
        pass
    
    def block_path(self, room1: str, room2: str, duration_seconds: int = None):
        """
        Temporarily block a path (e.g., human using doorway).
        """
        pass
    
    def find_shortest_path(self, start_room: str, end_room: str, 
                          carrying_load: bool = False) -> Tuple[List[str], float]:
        """
        Find shortest path considering current robot state.
        Use Dijkstra's algorithm with energy-aware costs.
        """
        pass
    
    def find_all_items_of_type(self, item_type: str) -> Dict[str, Set[str]]:
        """
        Find all rooms containing specific item type.
        Returns: {room_id: {item_ids}}
        """
        pass
    
    def plan_collection_route(self, item_type: str, start_room: str) -> Tuple[List[str], float]:
        """
        Plan optimal route to collect all items of given type.
        Solve Traveling Salesman-like problem with approximation.
        """
        pass
    
    def detect_room_clusters(self) -> List[Set[str]]:
        """
        Identify clusters of connected rooms (e.g., upstairs vs downstairs).
        Use connected components algorithm.
        """
        pass

class CORINavigationPlanner:
    def __init__(self, house_graph: HouseholdGraph):
        self.graph = house_graph
        self.current_room = None
        self.energy_level = 100.0  # Percentage
        self.carrying_capacity = 0  # Number of items currently carrying
        self.max_capacity = 5
        self.movement_history = []  # Track where CORI has been
    
    def update_position(self, new_room: str):
        """Update CORI's current position and movement history."""
        pass
    
    def plan_laundry_collection_mission(self, target_rooms: List[str]) -> Dict:
        """
        Plan comprehensive laundry collection across multiple rooms.
        
        Returns strategy including:
        - Room visit order
        - Expected energy consumption
        - Number of trips needed
        - Estimated completion time
        """
        pass
    
    def adaptive_pathfinding(self, destination: str) -> List[str]:
        """
        Find path that adapts to:
        - Current energy level
        - Blocked passages
        - Carrying load
        - Time of day preferences (avoid bedrooms at night)
        """
        pass
    
    def detect_navigation_patterns(self) -> Dict:
        """
        Analyze movement history to identify:
        - Most frequently used paths
        - Energy-efficient routes
        - Bottleneck locations
        - Optimal times for different rooms
        """
        pass
    
    def emergency_return_to_base(self) -> Tuple[List[str], bool]:
        """
        Find fastest path back to laundry room when:
        - Energy is critically low
        - Emergency stop is triggered
        - Carrying maximum load
        
        Returns: (path, is_reachable)
        """
        pass

# Advanced graph algorithms for household optimization
class HouseholdNetworkAnalyzer:
    def __init__(self, navigation_planner: CORINavigationPlanner):
        self.planner = navigation_planner
        self.graph = navigation_planner.graph
    
    def identify_critical_paths(self) -> List[Tuple[str, str]]:
        """
        Find edges whose removal would significantly impact navigation.
        Use edge betweenness centrality.
        """
        pass
    
    def suggest_layout_improvements(self) -> List[Dict]:
        """
        Analyze graph structure to suggest:
        - Additional connections that would improve efficiency
        - Rooms that could benefit from reorganization
        - Bottleneck reduction strategies
        """
        pass
    
    def simulate_obstacle_impact(self, blocked_edges: List[Tuple[str, str]]) -> Dict:
        """
        Simulate impact of temporary obstacles (furniture, people) on efficiency.
        """
        pass

# Test the navigation system
def test_cori_navigation():
    # Build house graph
    house = HouseholdGraph()
    
    # Add rooms
    rooms = [
        ("laundry", RoomType.LAUNDRY_ROOM, (0, 0)),
        ("hall1", RoomType.HALLWAY, (1, 0)),
        ("bedroom1", RoomType.BEDROOM, (2, 0)),
        ("bedroom2", RoomType.BEDROOM, (1, 1)),
        ("bathroom", RoomType.BATHROOM, (0, 1)),
    ]
    
    for room_id, room_type, coords in rooms:
        house.add_room(room_id, room_type, coords)
    
    # Add connections
    connections = [
        ("laundry", "hall1", EdgeType.DOORWAY, 2.0, 1.5),
        ("hall1", "bedroom1", EdgeType.DOORWAY, 1.5, 1.0),
        ("hall1", "bedroom2", EdgeType.DOORWAY, 1.5, 1.0),
        ("hall1", "bathroom", EdgeType.NARROW_PASSAGE, 1.0, 1.2),
    ]
    
    for r1, r2, edge_type, distance, energy in connections:
        house.add_connection(r1, r2, edge_type, distance, energy)
    
    # Add some laundry items
    house.room_contents["bedroom1"].add("red_shirt")
    house.room_contents["bedroom2"].add("blue_pants")
    house.room_contents["bathroom"].add("white_towel")
    
    # Test navigation
    planner = CORINavigationPlanner(house)
    planner.update_position("laundry")
    
    # Plan collection mission
    mission = planner.plan_laundry_collection_mission(["bedroom1", "bedroom2", "bathroom"])
    print(f"Collection mission plan: {mission}")
    
    # Test pathfinding
    path, cost = house.find_shortest_path("laundry", "bedroom1")
    print(f"Path to bedroom1: {path}, Energy cost: {cost}")
```

**Time Limit: 45 minutes**
**Follow-up**: How would you handle dynamic obstacles like people moving through the house?

---

## 8. Heaps (Priority Queues)

### Key Concepts:
- **Binary Heap**: Complete binary tree with heap property
- **Min-Heap**: Parent ≤ children, root is minimum
- **Max-Heap**: Parent ≥ children, root is maximum
- **Operations**: insert, extract-min/max, peek, heapify
- **Implementation**: Array-based with index calculations
- **Python heapq**: Min-heap implementation, negate values for max-heap
- **Time Complexities**: Insert/Delete O(log n), Peek O(1), Build O(n)
- **Applications**: Priority queues, Dijkstra's algorithm, sorting

### Notes:
-

### LeetCode Problems (AM):
_[Track morning problem-solving sessions]_

### ChatGPT Conversation (AM):
_[Record key discussions and clarifications from morning study]_

### ChatGPT Conversation (PM):
_[Evening review and deeper concept exploration]_

### LeetCode Problems (PM):
_[Evening practice problems and solutions]_

### CORI Specific Challenge:
**Problem: CORI's Dynamic Task Prioritization System**

Implement a sophisticated priority queue system for CORI that dynamically adjusts task priorities based on urgency, user preferences, energy levels, and learning from past performance.

```python
import heapq
import time
from dataclasses import dataclass
from typing import List, Dict, Optional
from enum import Enum

class TaskType(Enum):
    SORT_ITEM = "sort_item"
    FIND_ITEM = "find_item"
    NAVIGATE = "navigate"
    LEARN_PREFERENCE = "learn_preference"
    ANALYZE_HAMPER = "analyze_hamper"
    CHARGE_BATTERY = "charge_battery"

class UrgencyLevel(Enum):
    CRITICAL = 1    # Battery low, user waiting
    HIGH = 2        # User actively sorting
    MEDIUM = 3      # Regular tasks
    LOW = 4         # Background analysis
    BACKGROUND = 5  # Non-essential optimizations

@dataclass
class Task:
    task_id: str
    task_type: TaskType
    description: str
    base_priority: int  # 1-10, lower is higher priority
    urgency: UrgencyLevel
    estimated_duration: float  # seconds
    energy_required: float     # percentage of battery
    deadline: Optional[float]  # timestamp, None if no deadline
    dependencies: List[str]    # task_ids that must complete first
    user_initiated: bool       # True if user requested directly
    retry_count: int = 0
    created_time: float = None
    
    def __post_init__(self):
        if self.created_time is None:
            self.created_time = time.time()

class DynamicTaskScheduler:
    def __init__(self, robot_energy=100.0):
        self.task_heap = []  # Min-heap of (calculated_priority, task)
        self.task_registry = {}  # task_id -> Task
        self.completed_tasks = []  # History for learning
        self.blocked_tasks = set()  # Tasks waiting for dependencies
        self.robot_energy = robot_energy
        self.user_preference_weights = {
            'speed': 0.3,      # Prefer faster tasks
            'accuracy': 0.4,   # Prefer high-success tasks  
            'energy': 0.3      # Consider energy efficiency
        }
        self.performance_history = {}  # task_type -> [completion_times]
    
    def calculate_dynamic_priority(self, task: Task) -> float:
        """
        Calculate priority score considering multiple factors.
        Lower score = higher priority.
        """
        pass
    
    def add_task(self, task: Task) -> bool:
        """
        Add task to scheduler. Check dependencies and energy requirements.
        Return True if task was successfully added.
        """
        pass
    
    def get_next_task(self) -> Optional[Task]:
        """
        Get highest priority task that can be executed now.
        Consider dependencies, energy requirements, and deadlines.
        """
        pass
    
    def complete_task(self, task_id: str, success: bool, actual_duration: float):
        """
        Mark task as completed and update performance metrics.
        Learn from execution time and success rate.
        """
        pass
    
    def handle_task_failure(self, task_id: str, error_reason: str):
        """
        Handle failed task execution. Decide whether to retry,
        modify priority, or cancel based on failure type.
        """
        pass
    
    def update_robot_energy(self, new_energy: float):
        """
        Update robot energy level and re-prioritize tasks accordingly.
        """
        pass
    
    def emergency_reprioritize(self, emergency_type: str):
        """
        Handle emergency situations:
        - Low battery: prioritize charging
        - User waiting: boost user-initiated tasks
        - System error: prioritize diagnostic tasks
        """
        pass
    
    def optimize_schedule(self) -> List[Task]:
        """
        Return optimized task execution order considering:
        - Deadline constraints
        - Energy efficiency
        - Task dependencies
        - User preferences
        """
        pass

class AdaptiveLearningScheduler(DynamicTaskScheduler):
    def __init__(self, robot_energy=100.0):
        super().__init__(robot_energy)
        self.time_of_day_patterns = {}  # hour -> preferred_task_types
        self.user_activity_patterns = {}  # user_state -> task_priority_adjustments
        self.context_history = []  # Track context when tasks were successful
    
    def learn_from_context(self, current_context: Dict, completed_task: Task, success: bool):
        """
        Learn how context (time, user activity, energy level) affects task success.
        Adjust future prioritization based on patterns.
        """
        pass
    
    def predict_optimal_execution_time(self, task: Task) -> float:
        """
        Predict best time to execute task based on historical patterns.
        Consider user activity, energy cycles, success rates.
        """
        pass
    
    def adaptive_priority_adjustment(self, task: Task) -> float:
        """
        Adjust task priority based on learned patterns:
        - Time of day preferences
        - User behavior patterns
        - Historical success rates in similar contexts
        """
        pass
    
    def suggest_workflow_improvements(self) -> List[str]:
        """
        Analyze completed tasks to suggest workflow optimizations:
        - Task batching opportunities
        - Energy-efficient scheduling
        - Deadline management improvements
        """
        pass

class CORITaskManager:
    def __init__(self):
        self.scheduler = AdaptiveLearningScheduler()
        self.active_task = None
        self.task_execution_log = []
        self.user_feedback_history = []
    
    def process_user_request(self, request: str) -> List[Task]:
        """
        Parse user request and create appropriate tasks.
        Examples:
        - "find my red shirt" -> FindItem task with high priority
        - "sort the laundry basket" -> Multiple SortItem tasks
        - "analyze what's in the hamper" -> AnalyzeHamper task
        """
        pass
    
    def execute_task_cycle(self):
        """
        Main execution loop:
        1. Get next high-priority task
        2. Execute task
        3. Handle results and learn
        4. Update priorities based on new information
        """
        pass
    
    def handle_interruption(self, interruption_type: str, context: Dict):
        """
        Handle interruptions gracefully:
        - User needs immediate help
        - Low battery warning
        - Obstacle in path
        - New high-priority task added
        """
        pass
    
    def generate_progress_report(self) -> Dict:
        """
        Generate comprehensive progress report:
        - Tasks completed vs pending
        - Energy efficiency metrics
        - User satisfaction indicators
        - Learning progress summary
        """
        pass

# Complex test scenario
def test_adaptive_task_scheduling():
    manager = CORITaskManager()
    
    # Simulate a busy laundry day
    tasks = [
        Task("sort_001", TaskType.SORT_ITEM, "Sort red shirt", 3, 
             UrgencyLevel.MEDIUM, 15.0, 2.0, None, [], False),
        Task("find_001", TaskType.FIND_ITEM, "Find blue socks", 2,
             UrgencyLevel.HIGH, 30.0, 5.0, time.time() + 300, [], True),
        Task("charge_001", TaskType.CHARGE_BATTERY, "Recharge battery", 1,
             UrgencyLevel.CRITICAL, 1800.0, -50.0, None, [], False),
        Task("analyze_001", TaskType.ANALYZE_HAMPER, "Weekly analysis", 8,
             UrgencyLevel.BACKGROUND, 120.0, 3.0, None, [], False),
    ]
    
    # Add tasks to scheduler
    for task in tasks:
        manager.scheduler.add_task(task)
    
    # Simulate energy drop
    manager.scheduler.update_robot_energy(25.0)  # Low battery!
    
    # Test emergency reprioritization
    manager.scheduler.emergency_reprioritize("low_battery")
    
    # Get optimal execution order
    execution_order = manager.scheduler.optimize_schedule()
    print("Optimized execution order:")
    for i, task in enumerate(execution_order):
        print(f"{i+1}. {task.description} (Priority: {task.base_priority})")
    
    # Simulate task execution and learning
    for task in execution_order[:2]:  # Execute first two tasks
        success = True  # Assume success
        actual_time = task.estimated_duration * (0.8 + 0.4 * hash(task.task_id) % 1)
        manager.scheduler.complete_task(task.task_id, success, actual_time)
```

**Time Limit: 40 minutes**
**Follow-up**: How would you handle priority inversion when high-priority tasks depend on low-priority ones?

---

## 9. Sorting Algorithms

### Key Concepts:
- **Quick Sort**: O(n log n) average, O(n²) worst, in-place, unstable
- **Merge Sort**: O(n log n) always, stable, requires O(n) extra space
- **Insertion Sort**: O(n²), good for small/nearly sorted arrays
- **Bubble Sort**: O(n²), simple but inefficient, stable
- **Heap Sort**: O(n log n) always, in-place, unstable
- **Stability**: Maintains relative order of equal elements
- **Adaptive**: Performs better on partially sorted data

### Notes:
-

### LeetCode Problems (AM):
_[Track morning problem-solving sessions]_

### ChatGPT Conversation (AM):
_[Record key discussions and clarifications from morning study]_

### ChatGPT Conversation (PM):
_[Evening review and deeper concept exploration]_

### LeetCode Problems (PM):
_[Evening practice problems and solutions]_

### CORI Specific Challenge:
**Problem: CORI's Multi-Criteria Laundry Sorting System**

Implement a custom sorting system that sorts laundry items by multiple criteria (color similarity, fabric care requirements, user priority) with different sorting algorithms optimized for different scenarios.

```python
from typing import List, Tuple, Callable, Any
from dataclasses import dataclass
from enum import Enum
import math

class FabricType(Enum):
    COTTON = "cotton"
    WOOL = "wool" 
    SYNTHETIC = "synthetic"
    SILK = "silk"
    DENIM = "denim"
    LINEN = "linen"

class CareLevel(Enum):
    DELICATE = 1
    GENTLE = 2
    NORMAL = 3
    HEAVY_DUTY = 4

@dataclass
class LaundryItem:
    item_id: str
    color_rgb: Tuple[int, int, int]  # RGB color values
    fabric_type: FabricType
    care_level: CareLevel
    size: str  # S, M, L, XL
    user_priority: int  # 1-10, higher = more important
    soil_level: int     # 1-5, higher = dirtier
    last_washed: float  # timestamp
    wash_temperature: int  # Celsius
    
    def color_distance(self, other: 'LaundryItem') -> float:
        """Calculate Euclidean distance between colors in RGB space."""
        r1, g1, b1 = self.color_rgb
        r2, g2, b2 = other.color_rgb
        return math.sqrt((r1-r2)**2 + (g1-g2)**2 + (b1-b2)**2)

class CORISortingAlgorithms:
    """Collection of sorting algorithms optimized for different laundry scenarios."""
    
    @staticmethod
    def adaptive_quicksort(items: List[LaundryItem], 
                          key_func: Callable[[LaundryItem], Any],
                          threshold: int = 10) -> List[LaundryItem]:
        """
        Quick sort that switches to insertion sort for small subarrays.
        Optimized for real-time sorting when CORI picks up items.
        """
        def quicksort_recursive(arr, low, high):
            if high - low <= threshold:
                # Use insertion sort for small subarrays
                return CORISortingAlgorithms.insertion_sort_range(arr, low, high, key_func)
            
            if low < high:
                # Your quicksort implementation here
                pass
        
        if not items:
            return items
        
        items_copy = items.copy()
        quicksort_recursive(items_copy, 0, len(items_copy) - 1)
        return items_copy
    
    @staticmethod
    def insertion_sort_range(items: List[LaundryItem], start: int, end: int,
                           key_func: Callable[[LaundryItem], Any]) -> List[LaundryItem]:
        """Insertion sort for a specific range in the array."""
        pass
    
    @staticmethod
    def stable_merge_sort(items: List[LaundryItem], 
                         key_func: Callable[[LaundryItem], Any]) -> List[LaundryItem]:
        """
        Stable merge sort for maintaining order of equal priority items.
        Important when user has specific preferences for similar items.
        """
        def merge(left, right):
            result = []
            i = j = 0
            
            while i < len(left) and j < len(right):
                if key_func(left[i]) <= key_func(right[j]):
                    result.append(left[i])
                    i += 1
                else:
                    result.append(right[j])
                    j += 1
            
            result.extend(left[i:])
            result.extend(right[j:])
            return result
        
        if len(items) <= 1:
            return items
        
        mid = len(items) // 2
        left = CORISortingAlgorithms.stable_merge_sort(items[:mid], key_func)
        right = CORISortingAlgorithms.stable_merge_sort(items[mid:], key_func)
        
        return merge(left, right)
    
    @staticmethod
    def counting_sort_by_care_level(items: List[LaundryItem]) -> List[LaundryItem]:
        """
        Counting sort optimized for care levels (small range of integer values).
        Fastest for sorting by care requirements.
        """
        pass
    
    @staticmethod
    def radix_sort_by_color(items: List[LaundryItem]) -> List[LaundryItem]:
        """
        Radix sort using RGB color values.
        Efficient for sorting large batches by color similarity.
        """
        pass

class MultiCriteriaSorter:
    def __init__(self):
        self.algorithms = CORISortingAlgorithms()
        self.performance_history = {}  # Track algorithm performance
    
    def sort_by_priority_matrix(self, items: List[LaundryItem], 
                               criteria_weights: dict) -> List[LaundryItem]:
        """
        Sort by multiple criteria with weighted importance.
        
        criteria_weights example:
        {
            'color_similarity': 0.3,
            'care_level': 0.4,
            'user_priority': 0.2,
            'soil_level': 0.1
        }
        """
        def composite_key(item: LaundryItem) -> float:
            score = 0.0
            
            # Color similarity (group similar colors together)
            if criteria_weights.get('color_similarity', 0) > 0:
                # Calculate how different this color is from a reference
                reference_color = (128, 128, 128)  # Gray as reference
                color_distance = math.sqrt(sum((a-b)**2 for a, b in zip(item.color_rgb, reference_color)))
                score += criteria_weights['color_similarity'] * color_distance
            
            # Care level (delicates first)
            if criteria_weights.get('care_level', 0) > 0:
                score += criteria_weights['care_level'] * item.care_level.value
            
            # User priority (higher priority first, so negate)
            if criteria_weights.get('user_priority', 0) > 0:
                score -= criteria_weights['user_priority'] * item.user_priority
            
            # Soil level (dirtier items first for pre-treatment)
            if criteria_weights.get('soil_level', 0) > 0:
                score -= criteria_weights['soil_level'] * item.soil_level
            
            return score
        
        # Choose algorithm based on dataset size and criteria
        if len(items) < 20:
            return self.algorithms.adaptive_quicksort(items, composite_key)
        else:
            return self.algorithms.stable_merge_sort(items, composite_key)
    
    def sort_for_washing_efficiency(self, items: List[LaundryItem]) -> List[List[LaundryItem]]:
        """
        Group and sort items into optimal washing loads.
        Returns list of loads, each optimally sorted.
        """
        # First, group by compatible wash settings
        wash_groups = self._group_by_wash_compatibility(items)
        
        # Then sort each group for optimal washing
        sorted_loads = []
        for group in wash_groups:
            # Sort each load by soil level (dirtiest first for pre-treatment)
            sorted_group = self.algorithms.stable_merge_sort(
                group, 
                lambda item: (-item.soil_level, item.care_level.value)
            )
            sorted_loads.append(sorted_group)
        
        return sorted_loads
    
    def _group_by_wash_compatibility(self, items: List[LaundryItem]) -> List[List[LaundryItem]]:
        """Group items that can be washed together safely."""
        pass
    
    def adaptive_sort_selection(self, items: List[LaundryItem], 
                              sorting_context: str) -> List[LaundryItem]:
        """
        Automatically select best sorting algorithm based on:
        - Dataset size
        - Sorting context (real-time vs batch)
        - Historical performance
        """
        context_algorithms = {
            'real_time_pickup': 'adaptive_quicksort',  # Fast response needed
            'batch_processing': 'stable_merge_sort',   # Stability important
            'care_level_only': 'counting_sort_by_care_level',  # Specialized
            'color_grouping': 'radix_sort_by_color'    # Color-specific
        }
        
        selected_algorithm = context_algorithms.get(sorting_context, 'adaptive_quicksort')
        
        # Execute selected algorithm and track performance
        start_time = time.time()
        
        if selected_algorithm == 'adaptive_quicksort':
            result = self.algorithms.adaptive_quicksort(items, lambda x: x.user_priority)
        elif selected_algorithm == 'stable_merge_sort':
            result = self.algorithms.stable_merge_sort(items, lambda x: x.user_priority)
        # Add other algorithm cases...
        
        execution_time = time.time() - start_time
        self._update_performance_history(selected_algorithm, len(items), execution_time)
        
        return result
    
    def _update_performance_history(self, algorithm: str, dataset_size: int, execution_time: float):
        """Track algorithm performance for future optimization."""
        if algorithm not in self.performance_history:
            self.performance_history[algorithm] = []
        
        self.performance_history[algorithm].append({
            'size': dataset_size,
            'time': execution_time,
            'efficiency': dataset_size / execution_time if execution_time > 0 else 0
        })

# Test the sorting system
def test_cori_sorting_system():
    # Create sample laundry items
    items = [
        LaundryItem("shirt_001", (255, 0, 0), FabricType.COTTON, CareLevel.NORMAL, "M", 8, 3, time.time()-86400, 40),
        LaundryItem("pants_001", (0, 0, 255), FabricType.DENIM, CareLevel.HEAVY_DUTY, "L", 6, 4, time.time()-172800, 60),
        LaundryItem("blouse_001", (255, 192, 203), FabricType.SILK, CareLevel.DELICATE, "S", 9, 1, time.time()-259200, 30),
        LaundryItem("socks_001", (255, 255, 255), FabricType.COTTON, CareLevel.NORMAL, "M", 3, 5, time.time()-345600, 40),
        LaundryItem("sweater_001", (128, 0, 128), FabricType.WOOL, CareLevel.GENTLE, "L", 7, 2, time.time()-432000, 30),
    ]
    
    sorter = MultiCriteriaSorter()
    
    # Test multi-criteria sorting
    criteria = {
        'color_similarity': 0.2,
        'care_level': 0.4,
        'user_priority': 0.3,
        'soil_level': 0.1
    }
    
    sorted_items = sorter.sort_by_priority_matrix(items, criteria)
    print("Multi-criteria sorted items:")
    for item in sorted_items:
        print(f"  {item.item_id}: Priority={item.user_priority}, Care={item.care_level.name}")
    
    # Test washing efficiency sorting
    wash_loads = sorter.sort_for_washing_efficiency(items)
    print(f"\nOptimal washing loads: {len(wash_loads)} loads")
    for i, load in enumerate(wash_loads):
        print(f"  Load {i+1}: {[item.item_id for item in load]}")
    
    # Test adaptive algorithm selection
    real_time_sorted = sorter.adaptive_sort_selection(items, 'real_time_pickup')
    batch_sorted = sorter.adaptive_sort_selection(items, 'batch_processing')
    
    print(f"\nPerformance history: {sorter.performance_history}")
```

**Time Limit: 40 minutes**
**Follow-up**: How would you optimize sorting when items are added dynamically during sorting?

---

## 10. Binary Search

### Key Concepts:
- **Prerequisite**: Array must be sorted
- **Algorithm**: Divide search space in half each iteration
- **Time Complexity**: O(log n)
- **Space Complexity**: O(1) iterative, O(log n) recursive
- **Variants**: Find exact value, first/last occurrence, insertion point
- **Applications**: Searching, finding bounds, optimization problems
- **Python**: `bisect` module for insertion points

### Notes:
-

### LeetCode Problems (AM):
_[Track morning problem-solving sessions]_

### ChatGPT Conversation (AM):
_[Record key discussions and clarifications from morning study]_

### ChatGPT Conversation (PM):
_[Evening review and deeper concept exploration]_

### LeetCode Problems (PM):
_[Evening practice problems and solutions]_

### CORI Specific Challenge:
**Problem: CORI's Efficient Object Search in Sorted Spatial Database**

Implement binary search variants to help CORI quickly locate objects in its spatial memory database, which is sorted by various criteria like position, last seen time, and confidence scores.

```python
import bisect
import time
from typing import List, Tuple, Optional, Callable
from dataclasses import dataclass
from enum import Enum

@dataclass
class SpatialObject:
    object_id: str
    x_position: float
    y_position: float
    z_position: float  # Height from ground
    color: str
    object_type: str
    confidence_score: float  # 0.0 to 1.0
    last_seen: float  # timestamp
    times_detected: int
    
    @property
    def distance_from_origin(self) -> float:
        return (self.x_position**2 + self.y_position**2 + self.z_position**2)**0.5

class SpatialSearchDatabase:
    def __init__(self):
        self.objects_by_x = []      # Sorted by x_position
        self.objects_by_confidence = []  # Sorted by confidence_score
        self.objects_by_time = []   # Sorted by last_seen
        self.objects_by_distance = []  # Sorted by distance from origin
        self.object_lookup = {}     # object_id -> SpatialObject
    
    def add_object(self, obj: SpatialObject):
        """Add object to all sorted lists maintaining order."""
        # Add to lookup
        self.object_lookup[obj.object_id] = obj
        
        # Insert into sorted lists using binary search for position
        bisect.insort(self.objects_by_x, obj, key=lambda x: x.x_position)
        bisect.insort(self.objects_by_confidence, obj, key=lambda x: -x.confidence_score)  # Descending
        bisect.insort(self.objects_by_time, obj, key=lambda x: x.last_seen)
        bisect.insort(self.objects_by_distance, obj, key=lambda x: x.distance_from_origin)
    
    def find_objects_in_x_range(self, min_x: float, max_x: float) -> List[SpatialObject]:
        """Find all objects within x-coordinate range using binary search."""
        pass
    
    def find_objects_above_confidence(self, min_confidence: float) -> List[SpatialObject]:
        """Find all objects with confidence above threshold."""
        pass
    
    def find_recently_seen_objects(self, time_threshold: float) -> List[SpatialObject]:
        """Find objects seen after given timestamp."""
        pass
    
    def find_nearest_objects_to_position(self, x: float, y: float, z: float, 
                                       count: int = 5) -> List[SpatialObject]:
        """
        Find 'count' nearest objects to given position.
        Use binary search on distance-sorted list.
        """
        pass
    
    def binary_search_custom_predicate(self, predicate: Callable[[SpatialObject], bool],
                                     sorted_list: List[SpatialObject]) -> Optional[SpatialObject]:
        """
        Generic binary search for finding first object satisfying predicate.
        Works on any sorted list with any boolean condition.
        """
        pass

class CORIObjectSearchEngine:
    def __init__(self):
        self.database = SpatialSearchDatabase()
        self.search_history = []  # Track search patterns for optimization
        self.cache = {}  # Cache recent search results
    
    def smart_object_search(self, query: dict) -> List[SpatialObject]:
        """
        Intelligent search that combines multiple criteria efficiently.
        
        Query examples:
        {"color": "red", "min_confidence": 0.7, "near_position": (1.0, 2.0, 0.5)}
        {"object_type": "shirt", "recent_hours": 24, "x_range": (0, 5)}
        """
        pass
    
    def find_optimal_search_path(self, target_objects: List[str]) -> List[SpatialObject]:
        """
        Given list of object IDs to find, determine optimal search order
        to minimize total travel distance using binary search for positioning.
        """
        pass
    
    def predictive_object_location(self, object_id: str, 
                                 prediction_time: float) -> Tuple[float, float, float]:
        """
        Predict where object might be at future time based on movement patterns.
        Use binary search on historical position data.
        """
        pass
    
    def confidence_threshold_optimization(self, target_accuracy: float) -> float:
        """
        Find optimal confidence threshold that achieves target accuracy.
        Use binary search on confidence values to find best threshold.
        """
        pass
    
    def range_query_optimizer(self, dimension: str, target_count: int) -> Tuple[float, float]:
        """
        Find optimal range [min_val, max_val] that returns approximately target_count objects.
        Use binary search to narrow down the range efficiently.
        """
        pass

# Advanced binary search implementations
class AdvancedBinarySearch:
    @staticmethod
    def find_first_occurrence(arr: List[SpatialObject], target_value: float, 
                             key_func: Callable[[SpatialObject], float]) -> int:
        """Find index of first occurrence of target value."""
        left, right = 0, len(arr) - 1
        result = -1
        
        while left <= right:
            mid = (left + right) // 2
            mid_value = key_func(arr[mid])
            
            if mid_value == target_value:
                result = mid
                right = mid - 1  # Continue searching left for first occurrence
            elif mid_value < target_value:
                left = mid + 1
            else:
                right = mid - 1
        
        return result
    
    @staticmethod
    def find_last_occurrence(arr: List[SpatialObject], target_value: float,
                           key_func: Callable[[SpatialObject], float]) -> int:
        """Find index of last occurrence of target value."""
        pass
    
    @staticmethod
    def find_insertion_point(arr: List[SpatialObject], target_value: float,
                           key_func: Callable[[SpatialObject], float]) -> int:
        """Find index where target should be inserted to maintain sort order."""
        pass
    
    @staticmethod
    def binary_search_on_answer(check_function: Callable[[float], bool], 
                              min_val: float, max_val: float, 
                              precision: float = 1e-6) -> float:
        """
        Binary search on answer space for optimization problems.
        Find maximum value where check_function returns True.
        """
        pass
    
    @staticmethod
    def find_peak_element(arr: List[float]) -> int:
        """
        Find any peak element (element greater than neighbors) in O(log n).
        Peak element always exists in any array.
        """
        pass
    
    @staticmethod
    def search_rotated_sorted_array(arr: List[SpatialObject], target_value: float,
                                  key_func: Callable[[SpatialObject], float]) -> int:
        """
        Search in rotated sorted array (e.g., [4,5,6,7,0,1,2]).
        Useful when CORI's spatial data has circular coordinates.
        """
        pass

class CORINavigationOptimizer:
    def __init__(self, search_engine: CORIObjectSearchEngine):
        self.search_engine = search_engine
        self.movement_cost_cache = {}
    
    def binary_search_optimal_route(self, start_pos: Tuple[float, float, float],
                                  target_objects: List[str]) -> List[str]:
        """
        Use binary search to find optimal route through objects.
        Minimize total movement cost using binary search on route segments.
        """
        def calculate_route_cost(route_order: List[str]) -> float:
            total_cost = 0.0
            current_pos = start_pos
            
            for object_id in route_order:
                obj = self.search_engine.database.object_lookup[object_id]
                next_pos = (obj.x_position, obj.y_position, obj.z_position)
                distance = ((current_pos[0] - next_pos[0])**2 + 
                           (current_pos[1] - next_pos[1])**2 + 
                           (current_pos[2] - next_pos[2])**2)**0.5
                total_cost += distance
                current_pos = next_pos
            
            return total_cost
        
        # Use binary search approach to optimize route segments
        pass
    
    def find_optimal_search_radius(self, center_pos: Tuple[float, float, float],
                                 min_objects: int) -> float:
        """
        Binary search to find minimum radius that contains at least min_objects.
        """
        def count_objects_in_radius(radius: float) -> int:
            count = 0
            for obj in self.search_engine.database.object_lookup.values():
                distance = ((center_pos[0] - obj.x_position)**2 + 
                           (center_pos[1] - obj.y_position)**2 + 
                           (center_pos[2] - obj.z_position)**2)**0.5
                if distance <= radius:
                    count += 1
            return count
        
        # Binary search on radius
        left, right = 0.0, 100.0  # Assume max search radius of 100 units
        
        while right - left > 0.01:  # Precision threshold
            mid = (left + right) / 2
            if count_objects_in_radius(mid) >= min_objects:
                right = mid
            else:
                left = mid
        
        return right

# Test comprehensive binary search system
def test_cori_binary_search_system():
    # Create spatial database with sample objects
    database = SpatialSearchDatabase()
    
    sample_objects = [
        SpatialObject("red_shirt_001", 1.5, 2.0, 0.8, "red", "shirt", 0.95, time.time()-3600, 5),
        SpatialObject("blue_pants_001", 3.2, 1.5, 0.5, "blue", "pants", 0.87, time.time()-7200, 3),
        SpatialObject("white_sock_001", 0.8, 3.1, 0.2, "white", "sock", 0.78, time.time()-1800, 8),
        SpatialObject("black_shirt_002", 2.1, 0.9, 0.7, "black", "shirt", 0.92, time.time()-5400, 4),
        SpatialObject("green_towel_001", 4.0, 2.8, 1.2, "green", "towel", 0.83, time.time()-9000, 2),
    ]
    
    for obj in sample_objects:
        database.add_object(obj)
    
    search_engine = CORIObjectSearchEngine()
    search_engine.database = database
    
    # Test range searches
    print("Testing Binary Search Operations:")
    
    # Test 1: Find objects in X range
    x_range_objects = database.find_objects_in_x_range(1.0, 3.0)
    print(f"Objects in X range [1.0, 3.0]: {[obj.object_id for obj in x_range_objects]}")
    
    # Test 2: Find high confidence objects
    high_conf_objects = database.find_objects_above_confidence(0.85)
    print(f"High confidence objects (>0.85): {[obj.object_id for obj in high_conf_objects]}")
    
    # Test 3: Find recently seen objects
    recent_threshold = time.time() - 3600  # Last hour
    recent_objects = database.find_recently_seen_objects(recent_threshold)
    print(f"Recently seen objects: {[obj.object_id for obj in recent_objects]}")
    
    # Test 4: Advanced search with multiple criteria
    search_query = {
        "color": "red",
        "min_confidence": 0.9,
        "x_range": (1.0, 2.0)
    }
    results = search_engine.smart_object_search(search_query)
    print(f"Smart search results: {[obj.object_id for obj in results]}")
    
    # Test 5: Navigation optimization
    optimizer = CORINavigationOptimizer(search_engine)
    start_position = (0.0, 0.0, 0.0)
    target_ids = ["red_shirt_001", "blue_pants_001", "white_sock_001"]
    
    optimal_route = optimizer.binary_search_optimal_route(start_position, target_ids)
    print(f"Optimal collection route: {optimal_route}")
    
    # Test 6: Find optimal search radius
    center = (2.0, 2.0, 0.5)
    min_objects = 3
    optimal_radius = optimizer.find_optimal_search_radius(center, min_objects)
    print(f"Optimal search radius for {min_objects} objects: {optimal_radius:.2f}")
    
    # Test 7: Binary search on answer - find confidence threshold
    def accuracy_check(threshold: float) -> bool:
        # Simulate: higher threshold = higher accuracy but fewer detections
        return (threshold * 100) >= 85  # Target 85% accuracy
    
    optimal_threshold = AdvancedBinarySearch.binary_search_on_answer(
        accuracy_check, 0.0, 1.0, 0.01
    )
    print(f"Optimal confidence threshold: {optimal_threshold:.3f}")
```

**Time Limit: 35 minutes**
**Follow-up**: How would you handle binary search when the comparison function is expensive to compute?

---

## 11. Recursion & Backtracking

### Key Concepts:
- **Recursion**: Function calls itself with smaller subproblems
- **Base Case**: Termination condition to prevent infinite recursion
- **Recursive Case**: Problem broken down into smaller instances
- **Call Stack**: Memory overhead for recursive calls
- **Tail Recursion**: Optimization when recursive call is last operation
- **Backtracking**: Systematic exploration with undoing of choices
- **Applications**: Tree traversal, combinatorial problems, maze solving

### Notes:
-

### LeetCode Problems (AM):
_[Track morning problem-solving sessions]_

### ChatGPT Conversation (AM):
_[Record key discussions and clarifications from morning study]_

### ChatGPT Conversation (PM):
_[Evening review and deeper concept exploration]_

### LeetCode Problems (PM):
_[Evening practice problems and solutions]_

### CORI Specific Challenge:
**Problem: CORI's Path Planning with Obstacle Avoidance**

Implement recursive pathfinding and backtracking algorithms for CORI to navigate through a house while avoiding obstacles and finding optimal laundry collection routes.

```python
from typing import List, Tuple, Set, Optional, Dict
from enum import Enum
from dataclasses import dataclass

class CellType(Enum):
    EMPTY = "."
    OBSTACLE = "#"
    LAUNDRY_ITEM = "L"
    HAMPER = "H"
    CORI_START = "C"
    VISITED = "V"

@dataclass
class Position:
    x: int
    y: int
    
    def __hash__(self):
        return hash((self.x, self.y))
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

class HouseGrid:
    def __init__(self, grid: List[List[str]]):
        self.grid = [row[:] for row in grid]  # Deep copy
        self.rows = len(grid)
        self.cols = len(grid[0]) if grid else 0
        self.laundry_items = self._find_laundry_items()
        self.hampers = self._find_hampers()
        self.cori_start = self._find_cori_start()
    
    def _find_laundry_items(self) -> List[Position]:
        items = []
        for i in range(self.rows):
            for j in range(self.cols):
                if self.grid[i][j] == CellType.LAUNDRY_ITEM.value:
                    items.append(Position(i, j))
        return items
    
    def _find_hampers(self) -> List[Position]:
        hampers = []
        for i in range(self.rows):
            for j in range(self.cols):
                if self.grid[i][j] == CellType.HAMPER.value:
                    hampers.append(Position(i, j))
        return hampers
    
    def _find_cori_start(self) -> Position:
        for i in range(self.rows):
            for j in range(self.cols):
                if self.grid[i][j] == CellType.CORI_START.value:
                    return Position(i, j)
        return Position(0, 0)  # Default if not found
    
    def is_valid_position(self, pos: Position) -> bool:
        return (0 <= pos.x < self.rows and 
                0 <= pos.y < self.cols and 
                self.grid[pos.x][pos.y] != CellType.OBSTACLE.value)
    
    def get_neighbors(self, pos: Position) -> List[Position]:
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # right, down, left, up
        neighbors = []
        
        for dx, dy in directions:
            new_pos = Position(pos.x + dx, pos.y + dy)
            if self.is_valid_position(new_pos):
                neighbors.append(new_pos)
        
        return neighbors

class CORIPathfinder:
    def __init__(self, house_grid: HouseGrid):
        self.house = house_grid
        self.path_cache = {}  # Memoization for repeated path queries
    
    def find_path_recursive(self, start: Position, goal: Position, 
                          visited: Set[Position] = None) -> Optional[List[Position]]:
        """
        Recursive pathfinding using DFS with backtracking.
        Returns path from start to goal, or None if no path exists.
        """
        if visited is None:
            visited = set()
        
        # Base cases
        if start == goal:
            return [start]
        
        if start in visited or not self.house.is_valid_position(start):
            return None
        
        # Recursive case
        visited.add(start)
        
        for neighbor in self.house.get_neighbors(start):
            if neighbor not in visited:
                path = self.find_path_recursive(neighbor, goal, visited)
                if path is not None:
                    return [start] + path
        
        # Backtrack
        visited.remove(start)
        return None
    
    def find_all_paths_recursive(self, start: Position, goal: Position,
                               current_path: List[Position] = None,
                               all_paths: List[List[Position]] = None) -> List[List[Position]]:
        """
        Find ALL possible paths from start to goal using recursive backtracking.
        Useful for analyzing multiple route options.
        """
        if current_path is None:
            current_path = []
        if all_paths is None:
            all_paths = []
        
        # Add current position to path
        current_path.append(start)
        
        # Base case: reached goal
        if start == goal:
            all_paths.append(current_path[:])  # Copy the path
        else:
            # Recursive exploration
            for neighbor in self.house.get_neighbors(start):
                if neighbor not in current_path:  # Avoid cycles
                    self.find_all_paths_recursive(neighbor, goal, current_path, all_paths)
        
        # Backtrack
        current_path.pop()
        return all_paths
    
    def solve_laundry_collection_tsp(self, max_items: int = None) -> Tuple[List[Position], int]:
        """
        Solve Traveling Salesman Problem for optimal laundry collection.
        Use recursive backtracking to find shortest route visiting all items.
        """
        items_to_collect = self.house.laundry_items[:max_items] if max_items else self.house.laundry_items
        
        def tsp_recursive(current_pos: Position, remaining_items: Set[Position], 
                         current_distance: int, best_solution: Dict) -> None:
            
            # Pruning: if current distance already exceeds best, abandon this branch
            if current_distance >= best_solution.get('distance', float('inf')):
                return
            
            # Base case: all items collected
            if not remaining_items:
                # Return to nearest hamper
                hamper_distances = [self._calculate_distance(current_pos, hamper) 
                                  for hamper in self.house.hampers]
                if hamper_distances:
                    total_distance = current_distance + min(hamper_distances)
                    if total_distance < best_solution.get('distance', float('inf')):
                        best_solution['distance'] = total_distance
                        best_solution['path'] = current_pos
                return
            
            # Recursive case: try visiting each remaining item
            for item in list(remaining_items):
                distance_to_item = self._calculate_distance(current_pos, item)
                new_remaining = remaining_items - {item}
                
                tsp_recursive(item, new_remaining, 
                            current_distance + distance_to_item, best_solution)
        
        best_solution = {}
        tsp_recursive(self.house.cori_start, set(items_to_collect), 0, best_solution)
        
        return best_solution.get('path', []), best_solution.get('distance', float('inf'))
    
    def _calculate_distance(self, pos1: Position, pos2: Position) -> int:
        """Manhattan distance between two positions."""
        return abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y)

class LaundryCollectionPlanner:
    def __init__(self, pathfinder: CORIPathfinder):
        self.pathfinder = pathfinder
        self.collection_strategies = {}
    
    def plan_collection_with_constraints(self, energy_limit: int, 
                                       carrying_capacity: int) -> Dict:
        """
        Plan laundry collection considering robot constraints.
        Use recursive backtracking to find feasible collection strategy.
        """
        def recursive_planning(current_pos: Position, remaining_energy: int,
                             current_load: int, collected_items: Set[Position],
                             best_plan: Dict) -> None:
            
            # Base case: energy too low to return home
            min_return_energy = min(self._energy_to_hamper(current_pos, hamper) 
                                  for hamper in self.pathfinder.house.hampers)
            if remaining_energy < min_return_energy:
                return
            
            # Base case: carrying capacity full - must return to hamper
            if current_load >= carrying_capacity:
                self._plan_hamper_return(current_pos, collected_items, best_plan)
                return
            
            # Try collecting each remaining item
            remaining_items = set(self.pathfinder.house.laundry_items) - collected_items
            
            for item in remaining_items:
                energy_cost = self._calculate_energy_cost(current_pos, item)
                
                if energy_cost <= remaining_energy:
                    # Recursive exploration
                    new_collected = collected_items | {item}
                    recursive_planning(item, remaining_energy - energy_cost,
                                     current_load + 1, new_collected, best_plan)
            
            # Also consider returning to hamper now (even if not full)
            if collected_items:  # Only if we have items to deposit
                self._plan_hamper_return(current_pos, collected_items, best_plan)
        
        best_plan = {'total_items': 0, 'strategy': []}
        recursive_planning(self.pathfinder.house.cori_start, energy_limit, 0, set(), best_plan)
        
        return best_plan
    
    def _energy_to_hamper(self, pos: Position, hamper: Position) -> int:
        return self.pathfinder._calculate_distance(pos, hamper) * 2  # Assume 2 energy per move
    
    def _calculate_energy_cost(self, pos1: Position, pos2: Position) -> int:
        return self.pathfinder._calculate_distance(pos1, pos2) * 2
    
    def _plan_hamper_return(self, current_pos: Position, collected_items: Set[Position], 
                          best_plan: Dict) -> None:
        if len(collected_items) > best_plan['total_items']:
            best_plan['total_items'] = len(collected_items)
            best_plan['strategy'] = list(collected_items)

class AdvancedBacktrackingSolver:
    """Advanced backtracking problems for CORI's complex scenarios."""
    
    @staticmethod
    def solve_room_cleaning_schedule(rooms: List[str], time_slots: List[str],
                                   constraints: Dict[str, List[str]]) -> Optional[Dict[str, str]]:
        """
        Assign cleaning time slots to rooms with constraints.
        Example: kitchen can't be cleaned during meal times.
        """
        def is_valid_assignment(room: str, time_slot: str, assignment: Dict[str, str]) -> bool:
            # Check constraint violations
            forbidden_times = constraints.get(room, [])
            if time_slot in forbidden_times:
                return False
            
            # Check if time slot already assigned
            if time_slot in assignment.values():
                return False
            
            return True
        
        def backtrack_schedule(room_index: int, assignment: Dict[str, str]) -> bool:
            # Base case: all rooms scheduled
            if room_index >= len(rooms):
                return True
            
            current_room = rooms[room_index]
            
            # Try each time slot for current room
            for time_slot in time_slots:
                if is_valid_assignment(current_room, time_slot, assignment):
                    # Make assignment
                    assignment[current_room] = time_slot
                    
                    # Recursive exploration
                    if backtrack_schedule(room_index + 1, assignment):
                        return True
                    
                    # Backtrack
                    del assignment[current_room]
            
            return False
        
        assignment = {}
        if backtrack_schedule(0, assignment):
            return assignment
        return None
    
    @staticmethod
    def generate_all_laundry_combinations(items: List[str], 
                                        compatibility_rules: Dict[str, Set[str]]) -> List[List[str]]:
        """
        Generate all valid combinations of items that can be washed together.
        Use backtracking to explore all possibilities.
        """
        def is_compatible_group(group: List[str]) -> bool:
            for i, item1 in enumerate(group):
                for item2 in group[i+1:]:
                    if item2 not in compatibility_rules.get(item1, set()):
                        return False
            return True
        
        def generate_combinations(start_index: int, current_combination: List[str],
                                all_combinations: List[List[str]]) -> None:
            # Add current combination if valid and non-empty
            if current_combination and is_compatible_group(current_combination):
                all_combinations.append(current_combination[:])
            
            # Try adding each remaining item
            for i in range(start_index, len(items)):
                current_combination.append(items[i])
                
                # Only continue if still compatible
                if is_compatible_group(current_combination):
                    generate_combinations(i + 1, current_combination, all_combinations)
                
                # Backtrack
                current_combination.pop()
        
        all_combinations = []
        generate_combinations(0, [], all_combinations)
        return all_combinations

# Test the recursive and backtracking systems
def test_cori_pathfinding_system():
    # Create test house grid
    house_layout = [
        ["C", ".", ".", "#", "L"],
        [".", "#", ".", ".", "."],
        ["L", ".", "#", ".", "H"],
        [".", ".", ".", "L", "."],
        ["H", ".", ".", ".", "."]
    ]
    
    house = HouseGrid(house_layout)
    pathfinder = CORIPathfinder(house)
    planner = LaundryCollectionPlanner(pathfinder)
    
    # Test basic pathfinding
    start = house.cori_start
    first_item = house.laundry_items[0]
    
    path = pathfinder.find_path_recursive(start, first_item)
    print(f"Path to first laundry item: {[(p.x, p.y) for p in path] if path else 'No path found'}")
    
    # Test finding all paths
    all_paths = pathfinder.find_all_paths_recursive(start, first_item)
    print(f"Total possible paths: {len(all_paths)}")
    
    # Test TSP solution
    best_path, distance = pathfinder.solve_laundry_collection_tsp(max_items=2)
    print(f"Optimal collection route distance: {distance}")
    
    # Test constrained planning
    collection_plan = planner.plan_collection_with_constraints(energy_limit=50, carrying_capacity=2)
    print(f"Constrained collection plan: {collection_plan}")
    
    # Test advanced backtracking
    rooms = ["bedroom", "bathroom", "kitchen", "laundry"]
    time_slots = ["morning", "afternoon", "evening", "night"]
    constraints = {
        "kitchen": ["morning", "evening"],  # Can't clean during meal prep
        "bedroom": ["night"]  # Can't clean when people sleep
    }
    
    schedule = AdvancedBacktrackingSolver.solve_room_cleaning_schedule(rooms, time_slots, constraints)
    print(f"Optimal cleaning schedule: {schedule}")
    
    # Test laundry combinations
    laundry_items = ["white_shirt", "red_shirt", "blue_jeans", "white_socks"]
    compatibility = {
        "white_shirt": {"white_socks"},
        "white_socks": {"white_shirt"},
        "red_shirt": {"blue_jeans"},
        "blue_jeans": {"red_shirt"}
    }
    
    combinations = AdvancedBacktrackingSolver.generate_all_laundry_combinations(laundry_items, compatibility)
    print(f"Valid wash combinations: {combinations}")
```

**Time Limit: 45 minutes**
**Follow-up**: How would you optimize the backtracking to handle larger search spaces efficiently?

---

## 12. Dynamic Programming

### Key Concepts:
- **Optimal Substructure**: Problem can be broken into optimal subproblems
- **Overlapping Subproblems**: Same subproblems solved multiple times
- **Memoization**: Top-down approach with caching (recursive + cache)
- **Tabulation**: Bottom-up approach building solution iteratively
- **State Definition**: What parameters define a subproblem
- **Transition**: How to move from one state to another
- **Base Cases**: Known solutions for smallest subproblems

### Notes:
_[Space for personal notes and insights]_

### LeetCode Problems (AM):
_[Track morning problem-solving sessions]_

### ChatGPT Conversation (AM):
_[Record key discussions and clarifications from morning study]_

### ChatGPT Conversation (PM):
_[Evening review and deeper concept exploration]_

### LeetCode Problems (PM):
_[Evening practice problems and solutions]_

### CORI Specific Challenge:
**Problem: CORI's Optimal Task Scheduling with Dynamic Programming**

Implement dynamic programming solutions for CORI's complex optimization problems including energy management, task scheduling, and learning pattern optimization.

```python
from typing import List, Dict, Tuple, Optional
from functools import lru_cache
from dataclasses import dataclass
import time

@dataclass
class Task:
    task_id: str
    energy_cost: int
    completion_reward: int
    duration: int
    deadline: Optional[int]
    dependencies: List[str]

@dataclass
class LearningSession:
    session_id: str
    accuracy_improvement: float
    energy_required: int
    time_required: int
    prerequisites: List[str]

class CORIOptimalScheduler:
    def __init__(self, max_energy: int = 100):
        self.max_energy = max_energy
        self.memo_cache = {}
    
    def optimal_task_scheduling_knapsack(self, tasks: List[Task], available_energy: int) -> Tuple[List[str], int]:
        """
        Solve task scheduling as 0/1 knapsack problem.
        Maximize reward while staying within energy budget.
        
        DP State: dp[i][energy] = max reward using first i tasks with energy budget
        """
        n = len(tasks)
        
        # Create DP table
        dp = [[0 for _ in range(available_energy + 1)] for _ in range(n + 1)]
        
        # Fill DP table
        for i in range(1, n + 1):
            task = tasks[i-1]
            for energy in range(available_energy + 1):
                # Option 1: Don't include current task
                dp[i][energy] = dp[i-1][energy]
                
                # Option 2: Include current task (if energy allows)
                if energy >= task.energy_cost:
                    include_reward = dp[i-1][energy - task.energy_cost] + task.completion_reward
                    dp[i][energy] = max(dp[i][energy], include_reward)
        
        # Backtrack to find which tasks were selected
        selected_tasks = []
        energy = available_energy
        for i in range(n, 0, -1):
            if dp[i][energy] != dp[i-1][energy]:
                selected_tasks.append(tasks[i-1].task_id)
                energy -= tasks[i-1].energy_cost
        
        return selected_tasks[::-1], dp[n][available_energy]
    
    def minimize_completion_time_with_dependencies(self, tasks: List[Task]) -> Dict[str, int]:
        """
        Find minimum completion time considering task dependencies.
        
        DP State: dp[task_set] = minimum time to complete all tasks in set
        Use bitmask DP for subset enumeration.
        """
        n = len(tasks)
        task_map = {task.task_id: i for i, task in enumerate(tasks)}
        
        # Create dependency mask for each task
        dependency_masks = []
        for task in tasks:
            mask = 0
            for dep in task.dependencies:
                if dep in task_map:
                    mask |= (1 << task_map[dep])
            dependency_masks.append(mask)
        
        # DP with bitmask: dp[mask] = min time to complete tasks in mask
        dp = [float('inf')] * (1 << n)
        dp[0] = 0  # Base case: no tasks completed
        
        for mask in range(1 << n):
            if dp[mask] == float('inf'):
                continue
                
            for i in range(n):
                # Check if task i can be added (dependencies satisfied)
                if (mask & (1 << i)) == 0 and (mask & dependency_masks[i]) == dependency_masks[i]:
                    new_mask = mask | (1 << i)
                    dp[new_mask] = min(dp[new_mask], dp[mask] + tasks[i].duration)
        
        # Backtrack to find schedule
        schedule = {}
        mask = (1 << n) - 1  # All tasks completed
        current_time = dp[mask]
        
        while mask > 0:
            for i in range(n):
                if (mask & (1 << i)) and dp[mask] == dp[mask ^ (1 << i)] + tasks[i].duration:
                    schedule[tasks[i].task_id] = current_time - tasks[i].duration
                    current_time -= tasks[i].duration
                    mask ^= (1 << i)
                    break
        
        return schedule
    
    @lru_cache(maxsize=None)
    def max_learning_improvement_memoized(self, sessions: Tuple[LearningSession, ...], 
                                        time_budget: int, completed_sessions: Tuple[str, ...] = ()) -> float:
        """
        Maximize learning improvement within time budget using memoization.
        
        DP State: max_improvement(remaining_sessions, time_left, completed)
        """
        if time_budget <= 0 or not sessions:
            return 0.0
        
        max_improvement = 0.0
        
        for i, session in enumerate(sessions):
            # Check if prerequisites are satisfied
            if all(prereq in completed_sessions for prereq in session.prerequisites):
                if session.time_required <= time_budget:
                    # Include this session
                    remaining_sessions = sessions[:i] + sessions[i+1:]
                    new_completed = completed_sessions + (session.session_id,)
                    
                    improvement = (session.accuracy_improvement + 
                                 self.max_learning_improvement_memoized(
                                     remaining_sessions, 
                                     time_budget - session.time_required,
                                     new_completed))
                    
                    max_improvement = max(max_improvement, improvement)
        
        return max_improvement
    
    def optimal_energy_usage_over_time(self, tasks_by_hour: Dict[int, List[Task]], 
                                     hours: int) -> Dict[int, List[str]]:
        """
        Optimize energy usage across multiple time periods.
        
        DP State: dp[hour][energy] = max reward from hour to end with energy remaining
        """
        # Initialize DP table
        dp = [[0 for _ in range(self.max_energy + 1)] for _ in range(hours + 1)]
        task_choices = [[[] for _ in range(self.max_energy + 1)] for _ in range(hours + 1)]
        
        # Fill DP table backwards (from last hour to first)
        for hour in range(hours - 1, -1, -1):
            hour_tasks = tasks_by_hour.get(hour, [])
            
            for energy in range(self.max_energy + 1):
                # Option 1: Do nothing this hour (energy recharges)
                recharged_energy = min(self.max_energy, energy + 10)  # Assume +10 energy per hour rest
                dp[hour][energy] = dp[hour + 1][recharged_energy]
                task_choices[hour][energy] = []
                
                # Option 2: Do tasks this hour
                for task_combo in self._generate_task_combinations(hour_tasks, energy):
                    total_reward = sum(task.completion_reward for task in task_combo)
                    total_energy_cost = sum(task.energy_cost for task in task_combo)
                    remaining_energy = energy - total_energy_cost
                    
                    if remaining_energy >= 0:
                        total_value = total_reward + dp[hour + 1][remaining_energy]
                        if total_value > dp[hour][energy]:
                            dp[hour][energy] = total_value
                            task_choices[hour][energy] = [task.task_id for task in task_combo]
        
        # Extract optimal schedule
        schedule = {}
        current_energy = self.max_energy
        
        for hour in range(hours):
            chosen_tasks = task_choices[hour][current_energy]
            schedule[hour] = chosen_tasks
            
            # Update energy for next hour
            energy_used = sum(task.energy_cost for task in tasks_by_hour.get(hour, []) 
                            if task.task_id in chosen_tasks)
            current_energy = min(self.max_energy, current_energy - energy_used + 10)
        
        return schedule
    
    def _generate_task_combinations(self, tasks: List[Task], max_energy: int) -> List[List[Task]]:
        """Generate all valid combinations of tasks within energy budget."""
        combinations = []
        
        def backtrack(index: int, current_combo: List[Task], remaining_energy: int):
            if index >= len(tasks):
                combinations.append(current_combo[:])
                return
            
            # Option 1: Skip current task
            backtrack(index + 1, current_combo, remaining_energy)
            
            # Option 2: Include current task if energy allows
            if tasks[index].energy_cost <= remaining_energy:
                current_combo.append(tasks[index])
                backtrack(index + 1, current_combo, remaining_energy - tasks[index].energy_cost)
                current_combo.pop()
        
        backtrack(0, [], max_energy)
        return combinations

class CORILearningOptimizer:
    def __init__(self):
        self.learning_history = {}
        self.confidence_progression = {}
    
    def optimize_learning_sequence_dp(self, learning_opportunities: List[LearningSession],
                                    time_budget: int) -> Tuple[List[str], float]:
        """
        Find optimal sequence of learning sessions to maximize accuracy improvement.
        Consider prerequisites and diminishing returns.
        
        DP State: dp[time][skill_level] = max improvement achievable
        """
        n = len(learning_opportunities)
        
        # Map sessions to indices
        session_map = {session.session_id: i for i, session in enumerate(learning_opportunities)}
        
        # Create dependency graph
        can_take_after = [[] for _ in range(n)]
        for i, session in enumerate(learning_opportunities):
            for prereq in session.prerequisites:
                if prereq in session_map:
                    can_take_after[session_map[prereq]].append(i)
        
        # DP with bitmask for completed sessions
        memo = {}
        
        def dp(time_left: int, completed_mask: int) -> Tuple[float, List[str]]:
            if time_left <= 0:
                return 0.0, []
            
            if (time_left, completed_mask) in memo:
                return memo[(time_left, completed_mask)]
            
            best_improvement = 0.0
            best_sequence = []
            
            for i in range(n):
                if completed_mask & (1 << i):  # Already completed
                    continue
                
                session = learning_opportunities[i]
                
                # Check prerequisites
                prereq_satisfied = True
                for prereq in session.prerequisites:
                    if prereq in session_map:
                        prereq_idx = session_map[prereq]
                        if not (completed_mask & (1 << prereq_idx)):
                            prereq_satisfied = False
                            break
                
                if prereq_satisfied and session.time_required <= time_left:
                    # Take this session
                    new_mask = completed_mask | (1 << i)
                    future_improvement, future_sequence = dp(
                        time_left - session.time_required, new_mask
                    )
                    
                    total_improvement = session.accuracy_improvement + future_improvement
                    
                    if total_improvement > best_improvement:
                        best_improvement = total_improvement
                        best_sequence = [session.session_id] + future_sequence
            
            memo[(time_left, completed_mask)] = (best_improvement, best_sequence)
            return best_improvement, best_sequence
        
        return dp(time_budget, 0)
    
    def predict_confidence_progression_dp(self, color_detections: List[Tuple[str, bool]], 
                                        prediction_horizon: int) -> Dict[str, List[float]]:
        """
        Predict confidence score progression for each color using DP.
        
        DP State: confidence[color][detection_count] = expected confidence
        """
        colors = set(color for color, _ in color_detections)
        confidence_predictions = {}
        
        for color in colors:
            # Get historical data for this color
            color_history = [(correct,) for c, correct in color_detections if c == color]
            
            if not color_history:
                confidence_predictions[color] = [0.5] * prediction_horizon
                continue
            
            # DP to model confidence progression
            # State: dp[detections][correct_count] = probability
            max_detections = len(color_history) + prediction_horizon
            dp = [[0.0 for _ in range(max_detections + 1)] for _ in range(max_detections + 1)]
            
            # Base case: start with uniform probability
            dp[0][0] = 1.0
            
            # Fill DP table based on historical data
            for i, (correct,) in enumerate(color_history):
                for j in range(i + 1):
                    if dp[i][j] > 0:
                        if correct:
                            dp[i + 1][j + 1] += dp[i][j]
                        else:
                            dp[i + 1][j] += dp[i][j]
            
            # Predict future confidence progression
            current_detections = len(color_history)
            current_correct = sum(correct for correct, in color_history)
            
            confidence_sequence = []
            for future_step in range(prediction_horizon):
                # Calculate expected confidence at this step
                total_detections = current_detections + future_step + 1
                expected_confidence = (current_correct + 0.7 * (future_step + 1)) / total_detections
                confidence_sequence.append(min(1.0, max(0.0, expected_confidence)))
            
            confidence_predictions[color] = confidence_sequence
        
        return confidence_predictions

class CORIResourceOptimizer:
    """Advanced DP problems for resource optimization."""
    
    @staticmethod
    def optimal_battery_usage_schedule(tasks: List[Task], charging_opportunities: List[int],
                                     battery_capacity: int, initial_battery: int) -> Dict[str, int]:
        """
        Schedule tasks optimally considering battery constraints and charging opportunities.
        
        DP State: dp[time][battery] = max reward achievable from time with battery level
        """
        max_time = max(charging_opportunities) + 10 if charging_opportunities else 24
        
        # Initialize DP table
        dp = [[-1 for _ in range(battery_capacity + 1)] for _ in range(max_time + 1)]
        dp[0][initial_battery] = 0
        
        # Track choices for backtracking
        choices = [[None for _ in range(battery_capacity + 1)] for _ in range(max_time + 1)]
        
        for time in range(max_time):
            for battery in range(battery_capacity + 1):
                if dp[time][battery] == -1:
                    continue
                
                # Option 1: Wait/charge at this time
                if time in charging_opportunities:
                    new_battery = min(battery_capacity, battery + 20)  # Charge +20
                    if dp[time + 1][new_battery] < dp[time][battery]:
                        dp[time + 1][new_battery] = dp[time][battery]
                        choices[time + 1][new_battery] = "charge"
                else:
                    # Natural battery drain
                    new_battery = max(0, battery - 1)
                    if dp[time + 1][new_battery] < dp[time][battery]:
                        dp[time + 1][new_battery] = dp[time][battery]
                        choices[time + 1][new_battery] = "wait"
                
                # Option 2: Execute a task
                for task in tasks:
                    if task.energy_cost <= battery:
                        new_battery = battery - task.energy_cost
                        new_reward = dp[time][battery] + task.completion_reward
                        
                        if dp[time + task.duration][new_battery] < new_reward:
                            dp[time + task.duration][new_battery] = new_reward
                            choices[time + task.duration][new_battery] = task.task_id
        
        # Find best final state
        best_reward = -1
        best_final_state = None
        
        for time in range(max_time + 1):
            for battery in range(battery_capacity + 1):
                if dp[time][battery] > best_reward:
                    best_reward = dp[time][battery]
                    best_final_state = (time, battery)
        
        # Backtrack to build schedule
        schedule = {}
        time, battery = best_final_state
        
        while time > 0 and choices[time][battery] is not None:
            choice = choices[time][battery]
            if choice not in ["charge", "wait"]:
                # This was a task execution
                task = next(t for t in tasks if t.task_id == choice)
                schedule[choice] = time - task.duration
                time -= task.duration
                battery += task.energy_cost
            else:
                time -= 1
                if choice == "charge":
                    battery -= 20
                else:
                    battery += 1
        
        return schedule

# Complex test scenario
def test_cori_dynamic_programming():
    # Create sample tasks
    tasks = [
        Task("sort_reds", 15, 20, 30, 120, []),
        Task("sort_blues", 20, 25, 45, 180, []),
        Task("find_shirt", 10, 30, 20, 60, []),
        Task("analyze_hamper", 30, 15, 60, 300, ["sort_reds", "sort_blues"]),
        Task("update_database", 5, 10, 15, None, ["analyze_hamper"])
    ]
    
    scheduler = CORIOptimalScheduler(max_energy=100)
    
    # Test 1: Knapsack scheduling
    selected_tasks, max_reward = scheduler.optimal_task_scheduling_knapsack(tasks, 50)
    print(f"Optimal task selection (energy=50): {selected_tasks}, reward: {max_reward}")
    
    # Test 2: Dependency scheduling
    min_time_schedule = scheduler.minimize_completion_time_with_dependencies(tasks)
    print(f"Minimum time schedule: {min_time_schedule}")
    
    # Test 3: Learning optimization
    learning_sessions = [
        LearningSession("color_basics", 0.2, 5, 30, []),
        LearningSession("fabric_recognition", 0.3, 8, 45, ["color_basics"]),
        LearningSession("pattern_detection", 0.25, 10, 60, ["color_basics"]),
        LearningSession("advanced_sorting", 0.4, 15, 90, ["fabric_recognition", "pattern_detection"])
    ]
    
    optimizer = CORILearningOptimizer()
    learning_improvement, sequence = optimizer.optimize_learning_sequence_dp(learning_sessions, 180)
    print(f"Optimal learning sequence: {sequence}, improvement: {learning_improvement:.2f}")
    
    # Test 4: Multi-hour energy optimization
    tasks_by_hour = {
        0: [tasks[0], tasks[2]],  # Morning tasks
        1: [tasks[1]],           # Afternoon tasks  
        2: [tasks[3], tasks[4]]  # Evening tasks
    }
    
    energy_schedule = scheduler.optimal_energy_usage_over_time(tasks_by_hour, 3)
    print(f"Optimal energy schedule: {energy_schedule}")
    
    # Test 5: Confidence prediction
    color_detections = [
        ("red", True), ("red", True), ("red", False), ("red", True),
        ("blue", False), ("blue", True), ("blue", True),
        ("green", True), ("green", True), ("green", True)
    ]
    
    confidence_predictions = optimizer.predict_confidence_progression_dp(color_detections, 5)
    print(f"Confidence predictions: {confidence_predictions}")
    
    # Test 6: Battery optimization
    charging_times = [2, 6, 10]  # Hours when charging is available
    battery_schedule = CORIResourceOptimizer.optimal_battery_usage_schedule(
        tasks[:3], charging_times, 100, 80
    )
    print(f"Optimal battery schedule: {battery_schedule}")
```

**Time Limit: 45 minutes**
**Follow-up**: How would you handle dynamic programming when the state space becomes too large to store in memory?

---

