import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.BeforeEach

class MinHeapTest {

    @Test
    fun insert() {
        val heap = MinHeap<String>()
        heap.insert(data="test", heapNumber=3.2)
        heap.insert(data="booh", heapNumber=5.3)
        heap.insert(data="claw", heapNumber=1.0)
        heap.insert(data="tester", heapNumber=0.2)
        heap.insert(data="qq", heapNumber=0.5)
        heap.insert(data="anotherone", heapNumber=100.0)
        assertFalse(heap.insert(data="qq", heapNumber=40.0))
        assertEquals(heap.getMin(), "tester")
        assertEquals(heap.getMin(), "qq")
        assertEquals(heap.getMin(), "claw")
        assertEquals(heap.getMin(), "test")
        assertEquals(heap.getMin(), "booh")
        heap.insert(data="reentry", heapNumber=1000.0)
        assertEquals(heap.getMin(), "anotherone")
        assertEquals(heap.getMin(), "reentry")
        assertEquals(heap.getMin(), null)
    }

    @Test
    fun adjustPriority() {
        val heap = MinHeap<String>()
        heap.insert(data="test", heapNumber=3.2)
        heap.insert(data="booh", heapNumber=5.3)
        heap.insert(data="claw", heapNumber=1.0)
        heap.insert(data="tester", heapNumber=0.2)
        heap.adjustHeapNumber("booh", 0.0)
        assertEquals(heap.getMin(), "booh")
    }

    @Test
    fun isEmpty() {
        val heap = MinHeap<String>()
        assertTrue(heap.isEmpty())
        heap.insert("test", 30.0)
        assertFalse(heap.isEmpty())
        heap.getMin()
        assertTrue(heap.isEmpty())
    }
}

class MinPriorityQueueImplTest {

    private lateinit var priorityQueue: MyMinPriorityQueue<String>

    @BeforeEach
    fun setUp() {
        priorityQueue = MyMinPriorityQueue<String>()
    }

    @Test
    fun queueEmpty() {
        assertTrue(priorityQueue.isEmpty())
    }

    @Test
    fun addPriorityTest() {
        priorityQueue.addWithPriority("1", 1.0)
        assertFalse(priorityQueue.isEmpty())
    }

    @Test
    fun lowPriorityReturnTest() {
        priorityQueue.addWithPriority("1", 1.0)
        priorityQueue.addWithPriority("2", 0.5)
        assertEquals("2", priorityQueue.next())
    }

    @Test
    fun adjustPriorityTest() {
        priorityQueue.addWithPriority("1", 1.0)
        priorityQueue.addWithPriority("2", 2.0)
        priorityQueue.adjustPriority("2", 0.5)

        assertEquals("2", priorityQueue.next())
        assertEquals("1", priorityQueue.next())
    }
}

class DirectedWeightedGraphTest {

    private lateinit var graph: MyGraph<String>

    @BeforeEach
    fun setUp() {
        graph = MyGraph()
    }

    @Test
    fun graphEmpty() {
        assertTrue(graph.getVertices().isEmpty())
    }

    @Test
    fun addTest() {
        graph.addEdge("A", "B", 1.0)
        assertTrue(graph.getVertices().containsAll(listOf("A", "B")))
        assertEquals(1.0, graph.getEdges("A")["B"])
    }

    @Test
    fun getEdgeTest() {
        graph.addEdge("A", "B", 1.0)
        graph.addEdge("A", "C", 2.0)
        val edges = graph.getEdges("A")
        assertEquals(2, edges.size)
        assertEquals(1.0, edges["B"])
        assertEquals(2.0, edges["C"])
    }

    @Test
    fun clearTest() {
        graph.addEdge("A", "B", 1.0)
        graph.clear()
        assertTrue(graph.getVertices().isEmpty())
    }
}