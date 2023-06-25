using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MazeGenerator : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        maze = new MazeCell[mazeWidth, mazeHeight];

        for (int x = 0; x < mazeWidth; x++)
        {
            for (int y = 0; y < mazeHeight; y++)
            {
                maze[x, y] = new MazeCell(x, y);
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    [Range(5, 500)]
    public int mazeWidth = 5, mazeHeight = 5;
    public int startX, startY;
    MazeCell[,] maze;

    Vector2Int currentCell;


    List<Direction> directions = new List<Direction>
    {
        Direction.Up, Direction.Down, Direction.Left, Direction.Right
    };

    List<Direction> GetRandomDirections()
    {
        List<Direction> dir = new List<Direction>(directions);
        List<Direction> randDir = new List<Direction>();

        while (dir.Count > 0)
        {
            int rnd = Random.Range(0, dir.Count);
            randDir.Add(dir[rnd]);
            dir.RemoveAt(rnd);
        }

        return randDir;
    }

    bool IsCellValid(int x, int y)
    {
        if (x < 0 || y < 0 || x > mazeWidth - 1 || y > mazeHeight - 1 || maze[x, y].visited)
            return false;
        else
            return true;
    }

    Vector2Int CheckNeighbour()
    {
        List<Direction> randDir = GetRandomDirections();
        Vector2Int neighbour = currentCell;
        for (int i = 0; i < randDir.Count; i++)
        {
            switch (randDir[i])
            {
                case Direction.Up:
                    neighbour.y++;
                    break;
                case Direction.Down:
                    neighbour.y--;
                    break;
                case Direction.Left:
                    neighbour.x--;
                    break;
                case Direction.Right:
                    neighbour.x++;
                    break;

            }

            if (IsCellValid(neighbour.x, neighbour.y))
                return neighbour;
        }

        return currentCell;

    }

    void BreakWalls(Vector2Int primaryCell, Vector2Int secondaryCell)
    {
        if (primaryCell.x > secondaryCell.x)
        {
            maze[primaryCell.x, primaryCell.y].leftWall = false;
        }
    }

}

public enum Direction
{
    Up,
    Down,
    Left,
    Right
}

public class MazeCell
{
    public bool visited;
    public int x, y;
    public bool topWall;
    public bool leftWall;

    public Vector2Int position
    {
        get
        {
            return new Vector2Int(x, y);
        }
    }

    public MazeCell(int x, int y)
    {
        this.x = x;
        this.y = y;

        visited = false;

        // All walls are present until removed by the algorithm
        topWall = leftWall = true;
    }
}
