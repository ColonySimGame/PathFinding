// namespace PathFindingGrid;
//
// public abstract class Program
// {
//     private static void Main(string[] args)
//     {
//         var tiles = new Tile[4, 4];
//
//         var rows = tiles.GetLength(0);
//         var columns = tiles.GetLength(1);
//         for (var x = 0; x < rows; x++)
//         {
//             for (var y = 0; y < columns; y++)
//             {
//                 tiles[x, y] = new Tile(x, y);
//                 
//                 // Make some tiles unwalkable
//                 if (x == 1 && y == 1)
//                 {
//                     tiles[x, y].Walkable = false;
//                 }
//                 
//                 // Make some tiles slower
//                 if (x == 2 && y == 2)
//                 {
//                     tiles[x, y].SpeedModifier = 70;
//                 }
//             }
//         }
//         
//         var grid = new Grid(tiles);
//         Console.WriteLine(grid);
//
//         var map = new Map();
//         map.AddGrid(grid);
//         var algorithm = new Algorithm(map);
//         // algorithm.CreateCosts();
//     }
// }