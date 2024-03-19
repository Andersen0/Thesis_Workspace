document.addEventListener('DOMContentLoaded', (event) => {

    let lastStatusMessage = null;
    let statusData = [];
    var lastClass, lastDistance, state = 'idle';

    function updateStatusMessages() {
        console.log(state);
        const statusCircle = document.getElementById('status-circle');
        if (statusCircle) {
            if (state == 'idle') {
                statusCircle.style.backgroundColor = 'blue';
            } else if (state == 'moving') {
                statusCircle.style.backgroundColor = 'green';
            } else {
                statusCircle.style.backgroundColor = 'red';
            }
            lastStatusMessage = state;
        }
    }

    // Initialize SVG
    const statusChartElement = d3.select("#status-chart");
    if (!statusChartElement.empty()) {
        var svg = statusChartElement.append("svg")
            .attr("width", 1000)
            .attr("height", 300);
        console.log("HERE: ", svg.node());
    }

    // Rest of your D3 and other functions...

let xScale = d3.scaleLinear()
.range([20, 980]);

let yScale = d3.scaleLinear()
.domain([0, 2])  // The domain is the range of possible status values
.range([280, 20]);  // The range is the vertical dimension of the svg

let colorScale = d3.scaleOrdinal()
.domain([0, 1, 2])
.range(['red', 'blue', 'green']);  // 'stopping' = red, 'idle' = yellow, 'moving' = green



// Function to update the status graph
function updateStatusGraph() {

    // Recalculate domain for xScale
    xScale.domain([0, statusData.length - 1]);  // Adjust to cover the index of last element

    // Line generator
    let lineGenerator = d3.line()
    .x((d, i) => xScale(i))  // Use the index for the x position
    .y(d => yScale(d[0]));

    // Tooltip div
    const tooltip = d3.select("#tooltip");

    // Update circles
    svg.selectAll("circle")
    .data(statusData)
    .join(
        enter => enter.append("circle")
            .attr("cx", (d, i) => xScale(i))
            .attr("cy", d => yScale(d[0]))
            .attr("stroke", (d, i) => i === statusData.length - 1 ? "black" : "none")
            .attr("stroke-width", 5)
            .attr("r", 15)
            .attr("fill", d => colorScale(d[0]))
            .on("click", function(event, d) {
                // Toggle the tooltip
                if (tooltip.style("display") === "none") {
                    tooltip.style("display", "block")
                        .style("left", event.pageX + 10 + "px")
                        .style("top", event.pageY + 10 + "px")
                        .html(`Time Running [s]: ${d[1]} <br> Timestamp: ${d[2]}`);
                } else {
                    tooltip.style("display", "none");
                }
            }),
        update => update
            .attr("cx", (d, i) => xScale(i))
            .attr("cy", d => yScale(d[0]))
            .attr("r", (d, i) => i === statusData.length - 1 ? 15 : 10)
            .attr("stroke", (d, i) => i === statusData.length - 1 ? "black" : "none")
            .attr("stroke-width", 5)
            .attr("fill", d => colorScale(d[0])),
        exit => exit.remove()
    );

    // Update lines
    svg.selectAll("path")
        .data([statusData])  // Need to pass in data as array of arrays
        .join(
        enter => enter.append("path")
            .attr("d", lineGenerator)
            .attr("fill", "none")
            .attr("stroke", "black"),
        update => update
            .attr("d", lineGenerator)
        );

}



function updateMessages() {
Promise.all([
    fetch('/topic').then(response => response.json())
    ]).then(([topicMessages]) => {
    // We'll assume the most recent message is at the end of the array
    const mostRecentTopicMessage = topicMessages[topicMessages.length - 1];
    const state = JSON.parse(localStorage.getItem('state'));
    if (statusData.length > 30) {
    statusData.shift();
    }
    // Determine numerical value for status
    let statusValue;
    if (state === 'idle') {
    statusValue = 1;
    } else if (state === 'moving') {
    statusValue = 2;
    } else {
    statusValue = 0;
    // Assume any other status is 'stopping'
    }

     // Only append to statusData if the statusValue is different from the last one or if statusData is empty
     if (statusData.length === 0 || statusData[statusData.length - 1][0] !== statusValue) {
        // Append to data lists
        statusData.push([statusValue, mostRecentTopicMessage, Date.now()]); // Ensure you are pushing array of values
    }
    // Update the status graph
    updateStatusGraph();

});
}


// Update the table
function updateTable() {
Promise.all([
    fetch('/classes').then(response => response.json()),
    fetch('/distance').then(response => response.json())
]).then(([classes, distance]) => {
    var lastClass;
    if (classes.length === 0){
        lastClass = "none";
    } else {
        lastClass = classes[0][0] === 0 ? 'adult' : 'worker';
    }

    var lastDistance = distance[distance.length - 1];

    // Display the last class and last distance above the table
    // document.getElementById('lastClassDisplay').textContent = lastClass;
    // document.getElementById('lastDistanceDisplay').textContent = lastDistance;
    // document.getElementById('state').textContent = state;

    var cellId = lastDistance + '_' + lastClass;
    var cell = document.getElementById(cellId);

    // Reset all cells
    var cells = document.querySelectorAll('td');
    cells.forEach(cell => cell.classList.remove('active'));

    // Color the new active cell
    if (cell) {
        cell.classList.add('active');
        state = cell.textContent.trim().toLowerCase();
        localStorage.setItem('state', JSON.stringify(state));
    };
});}
    setInterval(updateStatusMessages, 1000);
    setInterval(updateMessages, 1000);
    setInterval(updateTable, 1000);
    setInterval(updateStatusGraph, 1000);
    updateTable(); // Call this function initially
});

