package drone;

import java.util.*;

/**
 * Navigate through a field, get to end point while covering all non-blocked tiles/cells
 * TODO  Use start position and closest rectangle spot, rather than upper left when weighting closest rectangle (use an expanding radius, and rectContains)
 * TODO: Optimization: Prioritize rectangles which are not accessible through subsequent rectangles/nodes
 * TODO: Optimization: Allow traversal over buildings as a route if faster
 *
 * TODO: BUG FIX: If no path between the next rectangle, infinite loop
 */

class Drone {
    private int [][] field;
    private int [] start; // col, row
    private int [] goal;
    private Integer [] curLoc;
    private int [] dimensions;  // Dimensions of field (x, y)
    private HashMap<Integer, Integer> rectangleAreas;
    private HashMap<Integer, Integer[][]> rectangles; // (-1: {{x1,y1},{x2,y2}})
    private HashMap<Integer, HashSet> traversalGraph;   // Graph of connected rectangle ids

    Drone(int [] dimen, int [] start, int [] end){
        dimensions = dimen.clone();
        field = new int[dimen[1]][dimen[0]];
        this.start = new int[2];
        goal = new int[2];
        curLoc = new Integer[2];
        this.start[0] = start[0];
        this.start[1] = start[1];
        this.goal[0] = end[0];
        this.goal[1] = end[1];
        this.curLoc[0] = start[0];
        this.curLoc[1] = start[1];
        traversalGraph = new HashMap<>();
        rectangles = new HashMap<>();
        rectangleAreas = new HashMap<>();

    }

    /**
     * Checks if a rectangle contains a point
     * @param rectId rectangle id (e.g -3 or -1); to be used in rectangles map
     * @param col col to locate
     * @param row row to locate
     * @return true if located inside rectangle
     */
    private boolean rectContains(int rectId, int col, int row){

        if (this.rectangles.containsKey(rectId)) {
            Integer[][] rectangle = this.rectangles.get(rectId);
            // Check if in width
            if (rectangle[1][0] >= col && col >= rectangle[0][0]) {
                // Check if in row
                if (rectangle[1][1] >= row && row >= rectangle[0][1]) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Finds neighbouring rectangles
     * @param rectId source rectangle's id in rectangles list
     * @return set of neighbouring rectId's
     */
    private HashSet<Integer> rectangleNeighbours(int rectId){
        HashSet<Integer> neighbours = new HashSet<>();

        Integer [][] rectangle = rectangles.get(rectId);
        int startCol = (rectangle[0][0] == 0) ? 0 : rectangle[0][0] - 1;
        int startRow = (rectangle[0][1] == 0) ? 0 : rectangle[0][1] - 1;
        int colUp = ( (rectangle[1][0] + 1) < dimensions[0]) ? rectangle[1][0] + 1 : dimensions[0] - 1;
        int rowUp = ( (rectangle[1][1] + 1) < dimensions[1]) ? rectangle[1][1] + 1 : dimensions[1] - 1;
        for (int col = startCol; col <=  colUp; col++) {
            for (int row = startRow; row <= rowUp; row++) {
                // Inside the rectangle
                if (row >= rectangle[0][1] && row <= rectangle[1][1] && col >= rectangle[0][0] && col <= rectangle[1][0])
                    continue;

                // Diagonals don't count
                if (col == (rectangle[1][0] + 1)) {
                    if (row == (rectangle[1][1] + 1)) {
                        // Bottom right diagonal
                        continue;
                    }  else if (row == (rectangle[0][1] - 1)) {
                        // Top right diagonal
                        continue;
                    }
                } else if ( col == (rectangle[0][0] -1)) {
                    if (row == (rectangle[0][1] - 1)) {
                        // Top Left
                        continue;
                    } else if (row == (rectangle[1][1] + 1)) {
                        // Bottom LEft
                        continue;
                    }
                }

                if (field[row][col] < 0) {
                    neighbours.add(field[row][col]);
                }
            }
        }
        return neighbours;
    }

    /**
     * Finds the rectangles that will be passed through to get to destination
     * @param rectIdSrc the starting rectangle
     * @param rectIdDest the destination rectangle
     */
    private String findPath(int rectIdSrc, int rectIdDest) {
        if (rectIdSrc == rectIdDest)
            return Integer.toString(-1 * rectIdSrc);

        ArrayList<Integer> visited = new ArrayList<>();
        PriorityQueue<String> paths = new PriorityQueue<>(); // Stores the path of rectangle ids

        paths.add(Integer.toString(-1*rectIdSrc));


        while (!paths.isEmpty()) {
            String edge = paths.remove();
            String [] pathIds = edge.split(",");
            HashSet neighbours = rectangleNeighbours(-1*Integer.parseInt(pathIds[pathIds.length-1]));

            for (Object neighbour : neighbours) {
                Integer nextNode = (Integer) neighbour * -1;
                if ( (nextNode * -1) == rectIdDest) {
                    // Goal rectangle reached
                    paths.add(edge + "," + nextNode.toString());
                    return (edge + "," + nextNode.toString());
                }
                if (!visited.contains(nextNode * -1)) {
                    visited.add(nextNode * -1);
                    paths.add(edge + "," + nextNode.toString());
                }
            }
        }
        return "0"; // Destination rectangle is not connected to the source (is an island)
    }

    /**
     * Initializes the traversal graph
     */
    private void initTraversalGraph() {
        for(int i = -1 ; i >= (-1*rectangles.size()); i--) {
            HashSet<Integer> neighbours = rectangleNeighbours(i);
            traversalGraph.put(i, neighbours);
        }
        Iterator it = traversalGraph.entrySet().iterator();
        while(it.hasNext()) {
            Map.Entry pair = (Map.Entry)it.next();
            System.out.println(pair.getKey() + " = " + pair.getValue());
        }
    }

    /**
     * Set a field cell to a value
     * @param x column location
     * @param y row location
     * @param value value to place in cell
     *              0 = clear, 1 = blocked,
     *              2 = start, 3 = end
     */
    void setField(int x, int y, int value) {
        field[y][x] = value;
    }

    /**
     * Print the field to stdout
     */
    void printField() {
        System.out.println();
        for (int [] column:
                field) {
            for (int cell:
                    column) {
                System.out.print(cell + " ");
            }
            System.out.println();
        }
    }

    /**
     * Uses the euclidean distance between two cells
     * @param cellOne
     * @param cellTwo
     * @return
     */
    private double cellDistanceWeight(Integer []cellOne, Integer []cellTwo){
        double weight = Math.pow(cellTwo[1] - cellOne[1],2) + Math.pow(cellTwo[0]^2 - cellOne[0]^2,2);
        return Math.sqrt(weight);
    }

    /**
     * Figures out the order to cover the rectangles based on degree, size, and distance
     */
    PriorityQueue<Integer[]> getTraversalOrder(){
        PriorityQueue<Integer[]> path = new PriorityQueue<>((nodeOne, nodeTwo) -> nodeOne[1] - nodeTwo[1]);

        // Initialize queue with priority on lower degree nodes
        Iterator it = traversalGraph.entrySet().iterator();
        while(it.hasNext()) {
            Map.Entry pair = (Map.Entry)it.next();
            Integer [] node = new Integer[2]; // [0] = rectId, [1] = weight
            node[0] = (Integer)pair.getKey();
            node[1] = traversalGraph.get(pair.getKey()).size();    // Initial weighting = degree
            path.add(node);
        }

        // Prioritize smaller nodes/rectangles if they have the same degree
        ArrayList<Integer[]> visited = new ArrayList<>();
        Integer [] prevNode = path.remove();
        while (!path.isEmpty()) {
            Integer [] curNode = path.remove();
            // Matching node degrees
            if (prevNode[1] == curNode[1]) {
                // Smaller one has priority; if equal then doesn't matter
                if (rectangleAreas.get(prevNode[0]) < rectangleAreas.get(curNode[0])) {
                    curNode[1]  += Math.abs(prevNode[0] - curNode[0]); // Difference in id = weight

                } else if (rectangleAreas.get(prevNode[0]) > rectangleAreas.get(curNode[0])) {
                    prevNode[1] += Math.abs(prevNode[0] - curNode[0]);
                }

            }
            visited.add(prevNode);
            prevNode = curNode;
        }
        visited.add(prevNode);

        // Add the newly weighted nodes to the queue
        for (Integer [] node:
             visited) {
            path.add(node);
        }
        visited.clear();

        // Prioritize nodes that are close to starting position
        while (!path.isEmpty()) {
            Integer[] node = path.remove();
            // Currently uses distance from start position -> topLeft rectangle corner
            int distance = (int) cellDistanceWeight(curLoc, rectangles.get(node[0])[0]);
            node[1] += distance;
            visited.add(node);
        }

        // Add the newly weighted nodes to the queue
        for (Integer [] node:
                visited) {
            if (rectContains(node[0], goal[0], goal[1])) {
                // Goal node should be completed last
                node[1] += 1000;
            }
            path.add(node);
        }

        return path;
    }

    /**
     * Navigate through the field
     * @return The traversed cells
     */
    ArrayList<Integer []> navigate() {
        ArrayList<Integer[]> traversed = new ArrayList<>();
        traversed.add(curLoc.clone());
        findAllRectangles();
        initTraversalGraph();
        PriorityQueue<Integer []> path = getTraversalOrder();
        printField();
        System.out.println();
        for (Integer[] x:
             path) {
            System.out.print(x[0]+ ":" + x[1] + " -> ");
        }
        System.out.println();

        while (!path.isEmpty()) {
            Integer [] dest = path.remove();

            // Navigate through rectangles to destination
            String rectanglePath = findPath(field[curLoc[1]][curLoc[0]], dest[0]);  // Non-negative rectId
            System.out.println(rectanglePath);
            // Figure direct path through rectangles/nodes
            if (rectanglePath == "0") {
                // Just fly directly to rectangle, since all paths block it
                // Can find a better path to the rectangle
                Integer [][] destCoords  = rectangles.get(dest[0]);
                traversed.addAll(traverseTo(destCoords[0][0], destCoords[0][1], true));

            } else {
                // Reachable via node traversal
                String [] pathIds = rectanglePath.split(",");
                for(int i = 0; i < pathIds.length; i++) {
                    // Travels the the nearest shared space
                    Integer destId = -1 * Integer.parseInt(pathIds[i]);
                    Integer [] joint = findJoiningPoint(field[curLoc[1]][curLoc[0]], destId);
                    traversed.addAll(traverseTo(joint[0], joint[1], false));
                }
            }

            // Go to the closest corner and begin covering the retangle
            Integer [] closestCorner = getClosestCorner(field[curLoc[1]][curLoc[0]]);
            traversed.addAll(traverseTo(closestCorner[0], closestCorner[1], false));
            traversed.addAll(traverseRect(field[curLoc[1]][curLoc[0]]));

        }
        // Covered all areas, now just move to goal if not already there
        traversed.addAll(traverseTo(goal[0], goal[1], true));
        return traversed;
    }

    /**
     * Finds the coordinates of the closest corner to the current position
     * @param rectId rectangle id currently in
     * @return {col, row} of corner
     */
    private Integer [] getClosestCorner(int rectId){
        Integer [] cornerCoords = new Integer[3];
        Integer [][] rectangle = rectangles.get(rectId);

        // Top left corner
        Double distance = cellDistanceWeight(curLoc, rectangle[0]);
        cornerCoords[0] = rectangle[0][0];
        cornerCoords[1] = rectangle[0][1];
        cornerCoords[2] = distance.intValue();

        distance = cellDistanceWeight(curLoc, rectangle[1]);

        // Bottom right corner
        if (cornerCoords[2] > distance) {
            cornerCoords[0] = rectangle[0][0];
            cornerCoords[1] = rectangle[0][1];
            cornerCoords[2] = distance.intValue();
        }

        // Bottom Left corner
        Integer [] corner = new Integer[2];
        corner[0] = rectangle[0][0];
        corner[1] = rectangle[1][1];
        distance = cellDistanceWeight(curLoc, corner);
        if (cornerCoords[2] > distance) {
            cornerCoords[0] = rectangle[0][0];
            cornerCoords[1] = rectangle[0][1];
            cornerCoords[2] = distance.intValue();
        }

        // Top Right corner
        corner[0] = rectangle[1][0];
        corner[1] = rectangle[0][1];
        distance = cellDistanceWeight(curLoc, corner);
        if (cornerCoords[2] > distance) {
            cornerCoords[0] = rectangle[0][0];
            cornerCoords[1] = rectangle[0][1];
            cornerCoords[2] = distance.intValue();
        }

        return cornerCoords;
    }

    /**
     * Find upper right most joining point between two rectangles
     * @param rectId1 id of first rectangle
     * @param rectId2 id of second rectangle
     * @return joint coordinates inside of second rectangle
     */
    private Integer [] findJoiningPoint(int rectId1, int rectId2){

        Integer [] joint = new Integer[2];
        joint[0] = 0;
        joint[1] = 0;
        if (rectId1 == rectId2)
            return curLoc;

        Integer[][] rect1 = rectangles.get(rectId1);
        int startCol = (rect1[0][0] == 0 ) ? 0 : rect1[0][0] - 1;
        int startRow = (rect1[0][1] == 0) ? 0 : rect1[0][1] - 1;
        int colUp = ( (rect1[1][0] + 1) < dimensions[0]) ? rect1[1][0] + 1 : dimensions[0] - 1;
        int rowDown = ( (rect1[1][1] + 1) < dimensions[1]) ? rect1[1][1] + 1 : dimensions[1] - 1;
        boolean foundJoint = false;
        Double curJointDistance = new Double(9999);

        for (int row = startRow; row <= rowDown; row++) {
            for (int col = startCol; col <= colUp; col++) {
                if (rectContains(rectId2, col, row)) {

                    // Checks if the previous joint location is closer than new one
                    Integer [] newJoint = new Integer[2];
                    newJoint[0] = col;
                    newJoint[1] = row;
                    Double distance = cellDistanceWeight(curLoc, newJoint);
                    if (curJointDistance > distance) {
                        joint[0] = col;
                        joint[1] = row;
                        curJointDistance = distance;
                    }

                    foundJoint = true;
                }

            }
            if (foundJoint) {
                return joint;
            }
        }
        return joint;
    }

    /**
     * Finds closest corner and moves towards the opposite one (e.g. Top left -> Bottom Right, BottomLeft -> Top Right)
     * @return List of visited positions
     */
    private ArrayList<Integer[]> traverseRect(int rectId){
        ArrayList<Integer[]> visited = new ArrayList<>();
        Integer [][] rectangle = rectangles.get(rectId);
        Integer [] oppositeCorner = new Integer[2];
        oppositeCorner[0] = (curLoc[0] == rectangle[0][0]) ? rectangle[1][0] : rectangle[0][0];
        oppositeCorner[1] = (curLoc[1] == rectangle[0][1]) ? rectangle[1][1] : rectangle[1][0];

        int area = (1 + rectangle[1][1] - rectangle[0][1]) * (1 + rectangle[1][0] - rectangle[0][0]);
        int cellCount = 1;

        while ( cellCount < area) {
            if (curLoc[0] == rectangle[1][0]) {
                while (curLoc[0] != rectangle[0][0]) {
                    visited.add(moveLeft());
                    cellCount++;
                }
            } else {
                while (curLoc[0] != rectangle[1][0]) {
                    visited.add(moveRight());
                    cellCount++;
                }
            }


            if (cellCount >= area) {
                return visited;
            }

            if (oppositeCorner[1] != rectangle[0][1]) {
                // Starting at top, go down
                visited.add(moveDown());
            } else {
                // If starting in bottom, go up
                visited.add(moveUp());
            }
            cellCount++;
        }

        return visited;
    }

    /**
     * Moves the current location 1 unit right
     * @return New current position
     */
    private Integer[] moveRight(){
        curLoc[0] += 1;
        return curLoc.clone();
    }

    /**
     * Moves the current location 1 unit left
     * @return New current position
     */
    private Integer[] moveLeft(){
        curLoc[0] -= 1;
        return curLoc.clone();
    }

    /**
     * Moves the current location 1 unit down
     * @return New current position
     */
    private Integer[] moveDown() {
        curLoc[1] += 1;
        return curLoc.clone();
    }

    /**
     * Moves the current location 1 unit up
     * @return New current position
     */
    private Integer[] moveUp(){
        curLoc[1] -= 1;
        return curLoc.clone();
    }

    /**
     * Navigate towards a target location
     * @param targetCol target column
     * @param targetRow target row
     * @param directPath flag to indicate whether to stay in own box, or allowed to fly directly there
     * @return List of points travelled to
     */
    private ArrayList<Integer[]> traverseTo(int targetCol, int targetRow, boolean directPath) {
        ArrayList<Integer[]> visited = new ArrayList<>();
        int srcId = field[curLoc[1]][curLoc[0]];
        int destId = field[targetRow][targetCol];

        while ( targetCol != curLoc[0] || targetRow != curLoc[1]) {
            if (targetRow > curLoc[1]) {
                for (int row = 1 + curLoc[1]; row <= targetRow; row++) {
                    // Point is downwards
                    if (field[row][curLoc[0]] > 0  && !directPath)
                        break;
                    if (directPath || field[row][curLoc[0]] == srcId || field[row][curLoc[0]] == destId) {
                        visited.add(moveDown());
                    } else {
                        break;
                    }
                }
            } else if (targetRow < curLoc[1]) {
                for (int row = curLoc[1] - 1; row >= targetRow; row--) {
                    if (field[row][curLoc[0]] > 0  && !directPath)
                        break;
                    // Point is upwards
                    if (directPath || field[row][curLoc[0]] == srcId || field[row][curLoc[0]] == destId) {
                        visited.add(moveUp());
                    } else {
                        break;
                    }
                }
            }

            if (targetCol > curLoc[0]) {
                for (int col = 1 + curLoc[0]; col <= targetCol; col++) {
                    if (field[curLoc[1]][col] > 0 && !directPath)
                        break;
                    // Point is towards right
                    if (directPath || field[curLoc[1]][col] == srcId || field[curLoc[1]][col] == destId) {
                        visited.add(moveRight());
                    } else {
                        break;
                    }
                }
            } else if (targetCol < curLoc[0]) {
                for (int col = curLoc[0]-1; col >= targetCol; col--) {
                    if (field[curLoc[1]][col] > 0 && !directPath)
                        break;
                    // Point is towards left
                    if (directPath || field[curLoc[1]][col] == srcId || field[curLoc[1]][col] == destId) {
                        visited.add(moveLeft());
                    } else {
                        break;
                    }
                }
            }
        }
        return visited;
    }

    /**
     * Finds the largest neighbour of a collection of neighbours
     * @return coordinates (closest square) of largest neighbour [0] = col, [1] = row, [2] = rectId, [3] = compass ( N = 0, NW = 7)
     */
    private Integer [] bestNeighbourBlock() {
        int curRow = this.curLoc[1];
        int curCol = this.curLoc[0];
        ArrayList<Integer[]> neighbours = new ArrayList<>();

        // Search the square around current cell for largest rectangle
        int radius = 1;

        // Loop until found a neighbour
        while (neighbours.size() == 0) {

            // Check North
            if ((curRow - radius) >= 0) {
                Integer[] neighbourCell = new Integer[4];
                neighbourCell[0] = curCol;
                neighbourCell[1] = curRow - radius;
                neighbourCell[2] = field[curRow - radius][curCol];
                neighbourCell[3] = 8 * radius; // Compass to say which direction to move and how far
                if (neighbourCell[2] < 0)
                    neighbours.add(neighbourCell);
            }

            // Check NorthEast
            if ((curCol + radius) < this.dimensions[0] && (curRow - radius) >= 0) {
                Integer[] neighbourCell = new Integer[4];
                neighbourCell[0] = curCol + radius;
                neighbourCell[1] = curRow - radius;
                neighbourCell[2] = field[curRow - radius][curCol + radius];
                neighbourCell[3] = 1 +  8 * radius;
                if (neighbourCell[2] < 0)
                    neighbours.add(neighbourCell);
            }

            // Check East
            if ((curCol + radius) < this.dimensions[0]) {
                Integer[] neighbourCell = new Integer[4];
                neighbourCell[0] = curCol + radius;
                neighbourCell[1] = curRow;
                neighbourCell[2] = field[curRow][curCol + radius];
                neighbourCell[3] = 2 + 8 * radius;
                if (neighbourCell[2] < 0)
                    neighbours.add(neighbourCell);
            }

            // Check SouthEast
            if ((curCol + radius) < this.dimensions[0] && (curRow + radius) < this.dimensions[1]) {
                Integer[] neighbourCell = new Integer[4];
                neighbourCell[0] = curCol + radius;
                neighbourCell[1] = curRow + radius;
                neighbourCell[2] = field[curRow + radius][curCol + radius];
                neighbourCell[3] = 3 + 8 * radius;
                if (neighbourCell[2] < 0)
                    neighbours.add(neighbourCell);
            }

            // Check South
            if ((curCol + radius) < this.dimensions[1]) {
                Integer[] neighbourCell = new Integer[4];
                neighbourCell[0] = curCol;
                neighbourCell[1] = curRow + radius;
                neighbourCell[2] = field[curRow + radius][curCol];
                neighbourCell[3] = 4 + 8 * radius;
                if (neighbourCell[2] < 0)
                    neighbours.add(neighbourCell);
            }

            // Check SouthWest
            if ((curCol - radius) >= 0 && (curRow + radius) < this.dimensions[1]) {
                Integer[] neighbourCell = new Integer[4];
                neighbourCell[0] = curCol - radius;
                neighbourCell[1] = curRow + radius;
                neighbourCell[2] = field[curRow + radius][curCol - radius];
                neighbourCell[3] = 5 + 8 * radius;
                if (neighbourCell[2] < 0)
                    neighbours.add(neighbourCell);
            }

            // Check West
            if ((curCol - radius) >= 0) {
                Integer[] neighbourCell = new Integer[4];
                neighbourCell[0] = curCol - radius;
                neighbourCell[1] = curRow;
                neighbourCell[2] = field[curRow][curCol - radius];
                neighbourCell[3] = 6 + 8 * radius;
                if (neighbourCell[2] < 0)
                    neighbours.add(neighbourCell);
            }

            // Check North West
            if ((curCol - radius) >= 0 && (curRow - radius) >= 0) {
                Integer[] neighbourCell = new Integer[4];
                neighbourCell[0] = curCol - radius;
                neighbourCell[1] = curRow - radius;
                neighbourCell[2] = field[curRow - radius][curCol - radius];
                neighbourCell[3] = 7 + 8 * radius;
                if (neighbourCell[2] < 0)
                    neighbours.add(neighbourCell);
            }
            radius++;

        }

        Integer [] bestNeighbour = new Integer[4];
        Arrays.fill(bestNeighbour, 0);

        // Which neighbour is the best
        for (Integer [] neighbour:
             neighbours) {
            if (bestNeighbour[2] == 0 || neighbour[2] > bestNeighbour[2]) {
                bestNeighbour = neighbour;
            }
        }

        return bestNeighbour;
    }

    /**
     * Finds the the largest combination of rectangles
     */
    private void findAllRectangles(){
        Integer [] rectangle = new Integer[5];
        Arrays.fill(rectangle, 0);
        Integer [] tempRect;
        int [] histoGram = new int[field[0].length];
        int numSpaces = 0;
        for (int y = 0; y < dimensions[1]; y++ ) {
            for (int i = 0; i < dimensions[0]; i++) {
                if (field[y][i] == 3 || field[y][i] < 0)
                    histoGram[i] = 0;
                else if (field[y][i] >= 0) {
                    histoGram[i] += 1;
                    numSpaces++;
                }
            }
            // Find largest rectangle in histogram
            tempRect = maxHist(histoGram);
            if (rectangle[3] < tempRect[3]) {
                rectangle[0] = tempRect[0]; // Col
                rectangle[1] = tempRect[1]; // Height
                rectangle[2] = tempRect[2]; // width
                rectangle[3] = tempRect[3]; // Area
                rectangle[4] = y - tempRect[1] + 1;
            }
        }

        if (numSpaces == 0) {
            return;
        }

        int rectId = -1*(rectangles.size() + 1);
        rectangleAreas.put( rectId, rectangle[3]);
        markRectangle(rectangle[4], rectangle[0], rectangle[1], rectangle[2], rectId);
        printField();
        findAllRectangles();
    }

    /**
     * Marks the field with rectangles using negative numbers; closer to 0 = larger rectangle
     * @param row the given row of the rectangle (top left)
     * @param col column of the rectangle
     * @param height height of rectangle
     * @param width width of rectangle
     * @param value what value to fill the cell with
     */
    private void markRectangle(int row, int col,int height, int width, int value){

        Integer [][] rectangle = new Integer[2][2];
        rectangle[0][0] = col; // top left column
        rectangle[0][1] = row; // Top row
        rectangle[1][0] = col + width - 1; // Bottom right column
        rectangle[1][1] = row + height - 1; // Bottom row

//        System.out.println("Top row: " + rectangle[0][1] + " | Left Col: " + rectangle[0][0] + " | Bottom Row: " + rectangle[1][1] + " | Right Col: " + rectangle[1][0]);

        for (int i = row; i <= rectangle[1][1];i++) {
            for (int j = col; j <= rectangle[1][0] ; j++) {
                field[i][j] = value;
            }
        }

        this.rectangles.put(value, rectangle);
    }

    /**
     * Find maximum rectangle in a given histogram/array
     * @param row row to find rectangle in
     * @return The rectangle's attributes [0] = column, [1] = height, [2] = width, [3] = area
     */
    private Integer[] maxHist(int row[]) {
        ArrayList<Integer> barIndexes = new ArrayList<>();
        int maxArea = 0;
        int localArea;
        int chain;

        Integer[] rectAtrs = new Integer[4];
        Arrays.fill(rectAtrs, 0);

        for (int i = 0; i < row.length; i++) {
            if (row[i] != 0) {
                // Add to chain
                barIndexes.add(i);
            } else {
                // Calculate area of sub-rectangles
                for (Integer index : barIndexes) {
                    chain = 1;
                    int curHeight = row[index];
                    int k = 1;
                    int startIndex = index;
                    // Find how far left rectangle is
                    while ((index - k) >= 0 && row[index - k] >= row[index]) {
                        chain++;
                        startIndex = index - k;
                        k++;
                    }
                    k = 1;
                    // Find how far right rectangle is
                    while( (index + k) < barIndexes.size() && (row[index+k] >= row[index])){
                        chain++;
                        k++;
                    }
                    localArea = chain * curHeight;
                    if (localArea > maxArea) {
                        maxArea = localArea;
                        rectAtrs[0] = startIndex;
                        rectAtrs[1] = row[index];
                        rectAtrs[2] = chain;
                        rectAtrs[3] = maxArea;
                    }
                }
                barIndexes.clear();
            }
        }
        if (!barIndexes.isEmpty()) {
            for (Integer index : barIndexes) {
                chain = 1;
                int curHeight = row[index];
                int k = 1;
                int startIndex = index;
                while ((index - k) >= 0 && row[index - k] >= row[index]) {
                    chain++;
                    startIndex = index - k;
                    k++;
                }
                k = 1;
                while( (index + k) < barIndexes.size() && (row[index+k] >= row[index])){
                    chain++;
                    k++;
                }
                localArea = chain * curHeight;
                if (localArea > maxArea) {
                    maxArea = localArea;

                    rectAtrs[0] = startIndex;
                    rectAtrs[1] = row[index];
                    rectAtrs[2] = chain;
                    rectAtrs[3] = maxArea;
                }
            }
        }
        return rectAtrs;
    }
}
