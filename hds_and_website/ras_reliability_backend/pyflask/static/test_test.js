// Click event handler for the buttons
function handleClick(buttonLabel) {
    const visualisation = d3.select("#visualisation");

    if (buttonLabel === "Robot") {
        // Toggle visibility of the visualisation SVG
        const isHidden = visualisation.style("display") === "none";
        visualisation.style("display", isHidden ? "block" : "none");
    } else {
        // Handle other button clicks here
    }
}

// Select the RobotContainer element
var RobotContainer = d3.select('#visualisation');

// Define the parameters for the rectangles
const rectWidth = 60; // Width of each rectangle
const rectHeight = 40; // Height of each rectangle
const spaceBetween = 20; // Space between the rectangles

// Calculate the position of the first rectangle to start in the middle of the container
const containerWidth = parseInt(RobotContainer.style('width'));
const containerHeight = parseInt(RobotContainer.style('height'));
const startX = (containerWidth - (rectWidth * 3 + spaceBetween * 2)) / 2;
const startY = (containerHeight - rectHeight) / 2;

// Create an array of positions for the rectangles
const positions = [
    { x: startX, y: startY },
    { x: startX + rectWidth + spaceBetween, y: startY },
    { x: startX + (rectWidth + spaceBetween) / 2, y: startY + rectHeight },
];

// Append three rectangles to the RobotContainer
RobotContainer.selectAll("rect")
    .data(positions)
    .enter()
    .append("rect")
    .attr("x", d => d.x)
    .attr("y", d => d.y)
    .attr("width", rectWidth)
    .attr("height", rectHeight)
    .style("fill", "blue"); // Set the color of the rectangles to blue
