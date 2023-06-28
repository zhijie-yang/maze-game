using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CharacterNavigation : MonoBehaviour
{
    public Transform exit; // The exit position of the maze
    public float gridSize = 5f; // Size of each grid
    public int mazeSize = 500; // Size of the maze
    public int observationRange = 5; // Range in which the character can observe obstacles
    public float heightThreshold = 1.0f; // Threadhold, beyond which is considered as obstacle
    public Vector3Int mazeOrigin;
    public float speed = 10.0f;
    public float closeThreshold = 0.1f;

    private Vector3 entryPosition; // The entry position of the maze
    private GridNode[,] grid; // 2D grid representing the maze
    private List<GridNode> openList; // List of nodes to be evaluated
    private List<GridNode> closedList; // List of nodes already evaluated
    private bool[,] obstacleGrid; // 2D grid representing the obstacle positions
    private bool isFindingPath; // Flag to check if path finding is in progress
    private Terrain terrain;
    [SerializeField]
    private List<Vector3> path = new();
    private Vector3 currObjectiveWaypoint;

    private CharacterController characterController; // Reference to the CharacterController component

    void Start()
    {
        terrain = Terrain.activeTerrain;
        entryPosition = transform.position; // Set the entry position to the character's starting position
        characterController = GetComponent<CharacterController>(); // Get the CharacterController component
        currObjectiveWaypoint = entryPosition;
        InitializeGrid(); // Initialize the grid
        FindPath(); // Find the path to the exit
    }

    void Update()
    {
        if (isFindingPath) return; // Exit if path finding is in progress

        // Check if the character has reached the exit
        if (Vector3.Distance(transform.position, exit.position) < gridSize / 2)
        {
            Debug.Log("Exit reached!");
            return;
        }

        // Move the character along the path
        MoveCharacter();
    }

    bool IsObstacle(Vector3 position, float heightThreshold)
    {
        float height_c = terrain.SampleHeight(position);
        float height_tl = terrain.SampleHeight(new Vector3(
                                                           position.x - gridSize / 2,
                                                           position.y,
                                                           position.z + gridSize / 2));
        float height_tr = terrain.SampleHeight(new Vector3(
                                                           position.x + gridSize / 2,
                                                           position.y,
                                                           position.z + gridSize / 2));
        float height_bl = terrain.SampleHeight(new Vector3(
                                                           position.x - gridSize / 2,
                                                           position.y,
                                                           position.z - gridSize / 2));
        float height_br = terrain.SampleHeight(new Vector3(
                                                           position.x + gridSize / 2,
                                                           position.y,
                                                           position.z - gridSize / 2));
        return !(height_c <= heightThreshold && height_tl < heightThreshold &&
                 height_tr <= heightThreshold && height_bl <= heightThreshold &&
                 height_br <= heightThreshold);
    }

    void InitializeGrid()
    {
        // Calculate the number of grids in the maze
        int numGrids = Mathf.CeilToInt(mazeSize / gridSize);

        // Initialize the grid and obstacle grid
        grid = new GridNode[numGrids, numGrids];
        obstacleGrid = new bool[numGrids, numGrids];

        // Populate the grid with nodes
        for (int x = 0; x < numGrids; x++)
        {
            for (int z = 0; z < numGrids; z++)
            {
                Vector3 nodePosition = new Vector3(
                    mazeOrigin.x + x * gridSize + gridSize / 2,
                    mazeOrigin.y,
                    mazeOrigin.z + z * gridSize + gridSize / 2
                );

                grid[x, z] = new GridNode(nodePosition, IsObstacle(nodePosition, heightThreshold), x, z);
            }
        }
    }

    void FindPath()
    {
        // Initialize the open and closed lists
        openList = new List<GridNode>();
        closedList = new List<GridNode>();

        // Calculate the start node indices
        int startX = Mathf.FloorToInt((transform.position.x - mazeOrigin.x) / gridSize);
        int startZ = Mathf.FloorToInt((transform.position.z - mazeOrigin.z) / gridSize);

        Debug.Log(string.Format("startX: {0}", startX));
        Debug.Log(string.Format("startZ: {0}", startZ));

        // Calculate the exit node indices
        int exitX = Mathf.FloorToInt((exit.position.x - mazeOrigin.x) / gridSize);
        int exitZ = Mathf.FloorToInt((exit.position.z - mazeOrigin.z) / gridSize);

        // Set the start and exit nodes
        GridNode startNode = grid[startX, startZ];
        GridNode exitNode = grid[exitX, exitZ];

        // Add the start node to the open list
        openList.Add(startNode);

        // Loop until a path is found or there are no more nodes to evaluate
        while (openList.Count > 0)
        {
            GridNode currentNode = GetLowestFScoreNode(openList);

            // Move the character to the current node position
            //characterController.Move(currentNode.position - transform.position);

            // Check if the current node is the exit node
            if (currentNode == exitNode)
            {
                GeneratePath(startNode, exitNode);
                Debug.Log("Exit reached!");
                break;
            }

            // Remove the current node from the open list and add it to the closed list
            openList.Remove(currentNode);
            closedList.Add(currentNode);

            // Get the neighboring nodes
            List<GridNode> neighbors = GetNeighbors(currentNode);

            foreach (GridNode neighbor in neighbors)
            {
                if (closedList.Contains(neighbor))
                    continue;

                // Calculate the cost to move to the neighbor node
                float moveCost = currentNode.gScore + CalculateDistance(currentNode, neighbor);

                if (moveCost < neighbor.gScore || !openList.Contains(neighbor))
                {
                    // Update the neighbor node's scores and parent
                    neighbor.gScore = moveCost;
                    neighbor.hScore = CalculateDistance(neighbor, exitNode);
                    neighbor.fScore = neighbor.gScore + neighbor.hScore;
                    neighbor.parent = currentNode;

                    if (!openList.Contains(neighbor))
                    {
                        // Add the neighbor node to the open list
                        openList.Add(neighbor);
                    }
                }
            }
        }

        isFindingPath = false; // Path finding is complete
    }

    void GeneratePath(GridNode startNode, GridNode endNode)
    {
        GridNode currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode.position);
            currentNode = currentNode.parent;
        }

        path.Reverse();
    }

    List<GridNode> GetNeighbors(GridNode node)
    {
        List<GridNode> neighbors = new List<GridNode>();
        int gridSizeX = grid.GetLength(0);
        int gridSizeZ = grid.GetLength(1);
        int x = node.xIndex;
        int z = node.zIndex;

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                int neighborX = x + i;
                int neighborZ = z + j;

                if (neighborX >= 0 && neighborX < gridSizeX && neighborZ >= 0 && neighborZ < gridSizeZ)
                {
                    GridNode neighbor = grid[neighborX, neighborZ];
                    if (!grid[neighborX, neighborZ].isObstacle)
                    {
                        neighbors.Add(neighbor);
                    }
                }
            }
        }

        return neighbors;
    }

    GridNode GetLowestFScoreNode(List<GridNode> nodeList)
    {
        GridNode lowestNode = nodeList[0];

        for (int i = 1; i < nodeList.Count; i++)
        {
            if (nodeList[i].fScore < lowestNode.fScore)
            {
                lowestNode = nodeList[i];
            }
        }

        return lowestNode;
    }

    float CalculateDistance(GridNode nodeA, GridNode nodeB)
    {
        float distanceX = Mathf.Abs(nodeA.xIndex - nodeB.xIndex);
        float distanceZ = Mathf.Abs(nodeA.zIndex - nodeB.zIndex);
        return distanceX + distanceZ;
    }

    bool isClose(Vector2 a, Vector2 b)
    {
        return Mathf.Abs(a.x - b.x) <= closeThreshold &&
               Mathf.Abs(a.y - b.y) <= closeThreshold;
    }

    void MoveCharacter()
    {
        Debug.Log(string.Format("currObjWaypt: {0} {1}", currObjectiveWaypoint.x, currObjectiveWaypoint.z));
        // Calculate the current grid indices
        int currentX = Mathf.FloorToInt((transform.position.x - mazeOrigin.x) / gridSize);
        int currentZ = Mathf.FloorToInt((transform.position.z - mazeOrigin.z) / gridSize);

        if (isClose(new Vector2(transform.position.x, transform.position.z),
                    new Vector2(currObjectiveWaypoint.x, currObjectiveWaypoint.z))
            && path.Count != 0)
        {
            Vector3 nextWaypoint = path[0];
            path.RemoveAt(0);
            currObjectiveWaypoint = nextWaypoint;
        }

        Vector3 moveDirection = currObjectiveWaypoint - transform.position;
        Debug.Log(string.Format("moveDirection: {0} {1}", moveDirection.x, moveDirection.z));
        moveDirection.y = 0f; // Prevent vertical movement
        moveDirection.Normalize();

        // Move the character using CharacterController's Move method
        characterController.Move(moveDirection * Time.deltaTime * speed);
    }

    private void OnDrawGizmos()
    {
        Gizmos.DrawWireCube(transform.position, new Vector3(gridSize, 1, gridSize));

        if (grid != null)
        {
            foreach (GridNode node in grid)
            {
                Gizmos.color = node.isObstacle ? Color.red : Color.white;
                //Gizmos.color = Color.white;
                if (node.xIndex == 87 && node.zIndex == 0)
                {
                    Gizmos.color = Color.green;
                }
                Gizmos.DrawCube(node.position, new Vector3(gridSize - 0.1f, 0.1f, gridSize - 0.1f));
            }
        }

        if (path != null)
        {
            Gizmos.color = Color.blue;
            foreach (Vector3 waypoint in path)
            {
                Gizmos.DrawSphere(waypoint + new Vector3(0.0f, 2.0f, 0.0f), gridSize * 0.5f);
            }
        }
    }


    // Class representing a node in the grid
    public class GridNode
    {
        public Vector3 position; // Position of the node
        public bool isObstacle;
        public int xIndex; // X index in the grid
        public int zIndex; // Z index in the grid
        public float gScore; // Cost from the start node
        public float hScore; // Cost to the exit node
        public float fScore; // Total cost
        public GridNode parent; // Parent node

        public GridNode(Vector3 pos, bool isObs, int x, int z)
        {
            position = pos;
            isObstacle = isObs;
            xIndex = x;
            zIndex = z;
        }
    }
}