package drone;

import javafx.fxml.FXML;
import javafx.scene.canvas.*;
import javafx.scene.control.SplitPane;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.Text;
import javafx.stage.FileChooser;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;

/**
 * TODO: Drawing of large canvas takes a while, maybe more efficient way.
 *
 * File input:
 * #columns, #rows,
 * startCol, startRow, endCol, endRow,
 * buildingTopLeftCol, buildingTopRow, buildingRightCol, building BotRow,
 *
 * Output:
 * Arrows indicate direction flown. Arrow turns from Red -> Blue as it takes steps
 */

public class Controller {
    private FileChooser fc = new FileChooser();
    private int [] dimensions = new int[2];
    private int [] start = new int [2];
    private int [] end = new int[2];
    private ArrayList<Integer []> buildings = new ArrayList<>();


    @FXML
    Text filenameText;

    @FXML
    SplitPane splitPane;

    @FXML
    Canvas canvas;

    /**
     * Opens the file and sets up the canvas' grid
     */
    @FXML
    void openFile() {
        buildings.clear();
        File file = fc.showOpenDialog(splitPane.getScene().getWindow());
        if (file != null) {
            try {
                BufferedReader br = new BufferedReader(new FileReader(file));
                filenameText.setText(file.getName());
                String line;
                String [] line_toks;
                if ((line = br.readLine()) != null){
                    // First line of file is dimensions
                    line_toks = line.split(",");
                    if (line_toks.length >= 2) {
//                        System.out.println(line_toks[0]);
                        dimensions[0] = Integer.parseInt(line_toks[0]);
                        dimensions[1] = Integer.parseInt(line_toks[1]);

                        if (dimensions[0] < 0 || dimensions[1] < 0){
                            System.out.println("Grid size cannot be negative.");
                            return;
                        }
                    } else {
                        System.out.println("Invalid grid size.");
                        return;
                    }
                }

                // Second line is startx, starty, endx, endy
                if ((line = br.readLine()) != null){
                    line_toks = line.split(",");
                    if (line_toks.length >= 4) {
                        start[0] = Integer.parseInt(line_toks[0]);
                        start[1] = Integer.parseInt(line_toks[1]);
                        if (start[0] < 0 || start[1] < 0) {
                            System.out.println("Invalid start coordinates.");
                            return;
                        } else if (start[0] > dimensions[0] || start[1] > dimensions[1]) {
                            System.out.println("Invalid start coordinates.");
                            return;
                        }

                        end[0] = Integer.parseInt(line_toks[2]);
                        end[1] = Integer.parseInt(line_toks[3]);

                        if (end[0] < 0 || end[1] < 0) {
                            System.out.println("Invalid end coordinates");
                            return;
                        } else if (end[0] > dimensions[0] || end[1] > dimensions[1]) {
                            System.out.println("Invalid end coordinates");
                            return;
                        }
                    } else {
                        System.out.println("Invalid start/end coordinates provided");
                        return;
                    }
                }

                // Load the building coordinates
                while ((line = br.readLine()) != null){
                    line_toks = line.split(",");
                    if (line_toks.length >= 4) {
                        Integer[] coords = new Integer[4];
                        coords[0] = Integer.parseInt(line_toks[0]);
                        coords[1] = Integer.parseInt(line_toks[1]);
                        coords[2] = Integer.parseInt(line_toks[2]);
                        coords[3] = Integer.parseInt(line_toks[3]);
                        buildings.add(coords);
                    }
                }

            } catch (Exception e) {
//                e.printStackTrace();
                System.out.println("Invalid file!");
            }

            // Create the environment in memory and draw environment on canvas
            Drone drone = new Drone(dimensions, start, end);
            initGrid(dimensions[0], dimensions[1]);
            for (Integer[] building:
                 buildings) {
                int height = Math.abs(building[1] - building[3]) + 1;
                int width = Math.abs(building[0] - building[2]) + 1;

                // Get top left most point
                int [] buildingCoord = new int[2];
                buildingCoord[0] = Math.min(building[0], building[2]);
                buildingCoord[1] = Math.min(building[1], building[3]);

                // Add buildings
                for (int i = buildingCoord[1]; i < (buildingCoord[1]+height); i++) {
                    for (int j = buildingCoord[0]; j< (buildingCoord[0]+width); j++) {
                        drone.setField(j, i,3);
                    }
                }
                drawRectangle(buildingCoord[0], buildingCoord[1], width, height, Color.BLACK);
            }

            drawRectangle(start[0], start[1], 1, 1, Color.GREY); // Start
            drawRectangle(end[0], end[1], 1, 1, Color.GREEN); // End


            // Start flying
//            drone.printField();
            ArrayList<Integer[]> traversed = drone.navigate();

            // Print traversal list for testing
            for (Integer [] cell:
                 traversed) {
                System.out.println(cell[0] + "," + cell[1]);
            }
            drawTraversed(traversed);
        }

    }

    private void drawTraversed(ArrayList<Integer[]> traversed){
        Integer[] prevNode;
        int maxSize = traversed.size();
        if (!traversed.isEmpty()){
            prevNode = traversed.get(0);
        } else {
            return;
        }
        int counter = 1;
        while (!traversed.isEmpty()) {
            Integer[] node = traversed.remove(0);

            GraphicsContext gc = canvas.getGraphicsContext2D();
            Font myFont = new Font("monospaced", canvas.getHeight() / dimensions[1]);
            gc.setFont(myFont);

            gc.setFill(Color.color( 1 - (double)counter/(double)maxSize, 0, (double)counter/(double)maxSize));

            Double[] start = getCellCoord(prevNode[0], prevNode[1]);

            // Get direction we are going
            if (node[0] == (prevNode[0]+1)) {
                // Moved right
                gc.fillText("\u2192", start[0] + canvas.getWidth()/dimensions[0]/4, start[1] + canvas.getHeight()/dimensions[1] - canvas.getHeight()/dimensions[1]/6);
            } else if (node[0] == (prevNode[0] - 1)) {
                // Moved left
                gc.fillText("\u2190", start[0]+ canvas.getWidth()/dimensions[0]/8, start[1] + canvas.getHeight()/dimensions[1] - canvas.getHeight()/dimensions[1]/6);
            } else if (node[1] == prevNode[1] + 1) {
                // Moved down
                gc.fillText("\u2193", start[0] + canvas.getWidth()/dimensions[0]/6, start[1]+ canvas.getHeight()/dimensions[1] - canvas.getHeight()/dimensions[1]/12);
            } else if (node[1] == prevNode[1] - 1){
                // Moved up
                gc.fillText("\u2191", start[0] + canvas.getWidth()/dimensions[0]/6, start[1] + canvas.getHeight()/dimensions[1] - canvas.getHeight()/dimensions[1]/6);
            }

            prevNode = node;
            counter++;

        }
    }

    /**
     * Draws the initial grid
     * @param width width of grid
     * @param height height of grid
     */
    private void initGrid(int width, int height) {
        GraphicsContext gc = canvas.getGraphicsContext2D();
        gc.clearRect(0,0,canvas.getWidth(),canvas.getHeight());
        gc.closePath();
        gc.setFill(Color.WHITE);
        gc.setStroke(Color.BLACK);
        gc.setLineWidth(2);

        // Draw outline of the grid
        gc.strokeLine(0,0, 0, canvas.getHeight());
        gc.strokeLine(0,canvas.getHeight(), canvas.getWidth(), canvas.getHeight());
        gc.strokeLine(canvas.getWidth(),canvas.getHeight(), canvas.getWidth(), 0);
        gc.strokeLine(canvas.getWidth(), 0, 0, 0);

        // Draw rows
        for (int i = 0; i < height; i++){
            double curHeight = i*canvas.getHeight()/height;
            gc.strokeLine(0, curHeight, canvas.getWidth(), curHeight);
        }

        // Draw columns
        for (int i = 1; i < width; i++){
            double curWidth = i*canvas.getWidth()/width;
            gc.strokeLine(curWidth, 0, curWidth, canvas.getHeight());
        }
    }

    /**
     * Gets cell coordinate in the canvas
     * @param x cell column
     * @param y cell width
     * @return canvas coordinates
     */
    private Double[] getCellCoord(int x, int y) {
        Double [] cell = new Double[2];
        cell[0] = x*(canvas.getWidth()/dimensions[0]);
        cell[1] = y*(canvas.getHeight()/dimensions[1]);
        return cell;
    }

    /**
     * Draws a rectangle of a given size
     * @param x starting column location
     * @param y starting row location
     * @param width width of rectangle
     * @param height height of rectangle
     * @param fillColour colour to fill the rectangle with
     */
    private void drawRectangle (int x, int y, int width, int height, Color fillColour) {
        GraphicsContext gc = canvas.getGraphicsContext2D();
        gc.setFill(fillColour);
        Double [] start = getCellCoord(x, y);
        gc.fillRect(start[0], start[1], width*(canvas.getWidth()/dimensions[0]), height*(canvas.getHeight()/dimensions[1]));
    }
}
