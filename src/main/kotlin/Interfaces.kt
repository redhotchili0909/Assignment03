/**
 * This ``Graph`` that represents a directed graph
 * @param VertexType the representation of a vertex in the graph
 */
interface Graph<VertexType> {
    /**
     * @return the vertices in the graph
     */
    fun getVertices(): Set<VertexType>

    /**
     * Add an
     */
    fun addEdge(from: VertexType, to: VertexType, cost: Double)

    /**
     *
     */
    fun getEdges(from: VertexType): Map<VertexType, Double>

    /**
     * Remove all edges and vertices from the graph
     */
    fun clear()
}

class MyGraph<VertexType> : Graph<VertexType> {
    private val vertices: MutableSet<VertexType> = mutableSetOf()
    private val edges: MutableMap<VertexType, MutableMap<VertexType, Double>> = mutableMapOf()

    override fun getVertices(): Set<VertexType> = vertices.toSet()

    override fun addEdge(from: VertexType, to: VertexType, cost: Double) {
        if (!vertices.contains(from)) vertices.add(from)
        if (!vertices.contains(to)) vertices.add(to)

        edges.getOrPut(from) { mutableMapOf() }[to] = cost
    }

    override fun getEdges(from: VertexType): Map<VertexType, Double> =
        edges[from] ?: emptyMap()

    override fun clear() {
        vertices.clear()
        edges.clear()
    }
}
/**
 * ``MinPriorityQueue`` maintains a priority queue where the lower
 *  the priority value, the sooner the element will be removed from
 *  the queue.
 *  @param T the representation of the items in the queue
 */
interface MinPriorityQueue<T> {
    /**
     * @return true if the queue is empty, false otherwise
     */
    fun isEmpty(): Boolean

    /**
     * Add [elem] with at level [priority]
     */
    fun addWithPriority(elem: T, priority: Double)

    /**
     * Get the next (lowest priority) element.
     * @return the next element in terms of priority.  If empty, return null.
     */
    fun next(): T?

    /**
     * Adjust the priority of the given element
     * @param elem whose priority should change
     * @param newPriority the priority to use for the element
     *   the lower the priority the earlier the element int
     *   the order.
     */
    fun adjustPriority(elem: T, newPriority: Double)
}

class MyMinPriorityQueue<T> : MinPriorityQueue<T> {
    private val minHeap = MinHeap<T>()

    override fun isEmpty(): Boolean = minHeap.isEmpty()

    override fun addWithPriority(elem: T, priority: Double) {
        minHeap.insert(elem, priority)
    }

    override fun next(): T? = minHeap.getMin()

    override fun adjustPriority(elem: T, newPriority: Double) {
        minHeap.adjustHeapNumber(elem, newPriority)
    }
}