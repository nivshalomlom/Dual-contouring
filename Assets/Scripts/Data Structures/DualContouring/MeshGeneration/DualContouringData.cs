namespace DualContouring
{

    // Delegates
    public delegate float Density(float x, float y, float z);
    public delegate bool Condition<T>(T item);

    /// <summary>
    /// A class to save all constant values required by dual contouring
    /// </summary>
    public static class DualContouringData
    {

        // Tables
        public static readonly int[,] cellProcFaceMask = 
        {
            {0, 4, 0},
            {1, 5, 0},
            {2, 6, 0},
            {3, 7, 0},
            {0, 2, 1},
            {4, 6, 1},
            {1, 3, 1},
            {5, 7, 1},
            {0, 1, 2},
            {2, 3, 2},
            {4, 5, 2},
            {6, 7, 2}
        };

        public static readonly int[,,] faceProcFaceMask = 
        {
            {
                {4, 0, 0},
                {5, 1, 0},
                {6, 2, 0},
                {7, 3, 0}
            },
            {
                {2, 0, 1},
                {6, 4, 1},
                {3, 1, 1},
                {7, 5, 1}
            },
            {
                {1, 0, 2},
                {3, 2, 2},
                {5, 4, 2},
                {7, 6, 2}
            }
        };

        public static readonly int[,] orders =
        {
            {0, 0, 1, 1},
            {0, 1, 0, 1},
        };

        public static readonly int[,,] faceProcEdgeMask = 
        {
            {
                {1, 4, 0, 5, 1, 1},
                {1, 6, 2, 7, 3, 1},
                {0, 4, 6, 0, 2, 2},
                {0, 5, 7, 1, 3, 2}
            },
            {
                {0, 2, 3, 0, 1, 0},
                {0, 6, 7, 4, 5, 0},
                {1, 2, 0, 6, 4, 2},
                {1, 3, 1, 7, 5, 2}
            },
            {
                {1, 1, 0, 3, 2, 0},
                {1, 5, 4, 7, 6, 0},
                {0, 1, 5, 0, 4, 1},
                {0, 3, 7, 2, 6, 1}
            }
        };

        public static readonly int[,,] edgeProcEdgeMask = 
        {
            {
                {3,2,1,0,0},
                {7,6,5,4,0}
            },
            {
                {5,1,4,0,1},
                {7,3,6,2,1}
            },
            {
                {6,4,2,0,2},
                {7,5,3,1,2}
            },
        };

        public static readonly int[,] edgevmap = 
        {
            {0, 4}, {1, 5}, {2, 6}, {3, 7}, // X - axis
            {0, 2}, {1, 3}, {4, 6}, {5, 7}, // Y - axis
            {0, 1}, {2, 3}, {4, 5}, {6, 7}  // Z - axis
        };

        public static readonly int[,] processEdgeMask = 
        {
            {3, 2, 1, 0},
            {7, 5, 6, 4},
            {11, 10, 9, 8}
        };

        public static readonly int[,] cellProcEdgeMask = 
        {
            {0, 1, 2, 3, 0},
            {4, 5, 6, 7, 0},
            {0, 4, 1, 5, 1},
            {2, 6, 3, 7, 1},
            {0, 2, 4, 6, 2},
            {1, 3, 5, 7, 2}
        };

    }

}
