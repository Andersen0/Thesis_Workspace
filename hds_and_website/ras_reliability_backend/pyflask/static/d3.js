
document.addEventListener("DOMContentLoaded", () => {
    
    
    const width = 800,
        height = 400,
        margin = { top: 20, right: 20, bottom: 20, left: 20 };

    const svg = d3.select("#chart")
        .append("svg")
        .attr("width", width + margin.left + margin.right)
        .attr("height", height + margin.top + margin.bottom)
        .append("g")
        .attr("transform", `translate(${margin.left},${margin.top})`);
       

    const subgraphWidth = width * 2 / 8,
        subgraphHeight = height * 1 / 5;

    const subgraph = svg.append("g")
        .attr("id", "subgraph")
        .attr("transform", `translate(${width - subgraphWidth - 20}, 0)`);

    subgraph.append("text")
        .style("font-size", "16px");

    svg.append('defs').append('marker')
        .attr("id", 'arrowhead')
        .attr('viewBox', '-0 -5 10 10')
        .attr('refX', 24)
        .attr('refY', 0)
        .attr('orient', 'auto')
        .attr('markerWidth', 3)
        .attr('markerHeight', 3)
        .attr('xoverflow', 'visible')
        .append('svg:path')
        .attr('d', 'M 0,-5 L 10 ,0 L 0,5')
        .attr('fill', 'none')
        .style('stroke', '#999');

    svg.append("text")
        .text("Robot Components")
        .attr("text-anchor", "middle")
        .attr("x", width / 2)
        .style("font-size", "20px");

    const triangleCenter = { x: width / 2, y: height / 2 };
    const distanceFromCenter = 100; // Adjust this value as needed for the size of your SVG.

    const dataset = {
        nodes: [
            { 
                id: 1, 
                name: 's_hds = 1', 
                label: 'Human detected in green zone', 
                group: 'states', 
                runtime: 20, 
                state: 'idle'
            },
            { 
                id: 2, 
                name: 's_hds = 2', 
                label: 'Human detected in yellow zone', 
                group: 'states', 
                runtime: 20, 
                state: 'stopped'
            },
            { 
                id: 0, 
                name: 's_hds = 0', 
                label: 'No human detected', 
                group: 'states', 
                runtime: 20, 
                state: 'moving'
            },
            { 
                id: 3, 
                name: 's_hds = 3', 
                label: 'error', 
                group: 'states', 
                runtime: 20, 
                state: 'error'
            }
        ],
        links: [
            { source: 0, target: 1, type: 'p_hds_green, s_human=1' },
            { source: 0, target: 2, type: 'p_hds_yellow, s_human=2' },
            { source: 1, target: 0, type: 's_human=0' },
            { source: 1, target: 2, type: 'p_hds_yellow, s_human=2' },
            { source: 2, target: 0, type: 's_human=0' },
            { source: 2, target: 1, type: 'p_hds_green, s_human=1' },
            { source: 0, target: 3, type: 'p_hds_red, s_human=3' },
            { source : 1, target: 3, type: 'p_hds_red, s_human=3' },
        ]
    };

    // const stateColor = d3.scaleOrdinal()
    //     .domain(['moving', 'idle', 'stopped'])
    //     .range(['#4CAF50', '#FFC107', '#F44336']);

    const colorScale = d3.scaleOrdinal(d3.schemeCategory10);

    const simulation = d3.forceSimulation()
    .force("link", d3.forceLink().id(d => d.id).distance(100))
    .force("charge", d3.forceManyBody().strength(-5000)) // You can adjust the strength for repulsion or attraction
    .force("center", d3.forceCenter(width / 2, height / 2))
    .force("collide", d3.forceCollide(20))  // Prevents node overlap
    .force("radial", d3.forceRadial(d => {
        // This is just an example. Adjust based on your specific criteria.
        if (d.group === 'states') return 150; 
        return 50;
    }, width / 2, height / 2));


    const drag = simulation => {
        function dragstarted(event, d) {
            if (!event.active) simulation.alphaTarget(0.3).restart();
            d.fx = d.x;
            d.fy = d.y;
        }
        
        function dragged(event, d) {
            d.fx = Math.max(17, Math.min(width - 17, event.x));
            d.fy = Math.max(17, Math.min(height - 17, event.y));
        }
        
        function dragended(event, d) {
            if (!event.active) simulation.alphaTarget(0);
            
            // Update the text position after the drag ends
            d3.select(nodeText.nodes()[d.index])
              .attr("dx", d.x + 25)
              .attr("dy", d.y + 5);
        }
        

        return d3.drag()
            .on("start", dragstarted)
            .on("drag", dragged)
            .on("end", dragended);
        
        
    }

    const link = svg.append("g")
        .attr("class", "links")
        .selectAll("line")
        .data(dataset.links)
        .enter()
        .append("line")
        .attr("stroke", "#999")
        .attr("stroke-width", 2)
        .attr('marker-end', 'url(#arrowhead)')
        .on("mouseover", function(d) {
            d3.select(this)
                .style("stroke-width", 5)
                .style("cursor", "pointer");
        })
        .on("mouseout", function(d) {
            d3.select(this)
                .style("stroke-width", 2)
                .style("cursor", "default");
        });

    const node = svg.append("g")
        .attr("class", "nodes")
        .selectAll("circle")
        .data(dataset.nodes)
        .enter()
        .append("circle")
        .attr("r", d => 17)
        .style("stroke", "grey")
        .style("stroke-opacity", 0.3)
        .style("fill", d => colorScale(d.group)) // Use the stateColor scale here
        .on("mouseover", function(d) {
            d3.select(this)
                .attr("r", 22)
                .style("stroke-width", 4)
                .style("cursor", "pointer");
        })
        .on("mouseout", function(d) {
            d3.select(this)
                .attr("r", 17)
                .style("stroke-width", 1)
                .style("cursor", "default");
        })
        .call(drag(simulation));

    node.append("title")
        .text(d => d.name);

    // Create text elements for node names
    const nodeText = svg.append("g")
        .attr("class", "node-texts")
        .selectAll("text")
        .data(dataset.nodes)
        .enter()
        .append("text")
        .attr("dx", d => d.x + 25) // Adjust this value for the text's x position relative to the circle
        .attr("dy", d => d.y + 5)  // Adjust this value for the text's y position relative to the circle
        .text(d => d.name);

    simulation.nodes(dataset.nodes); // Set nodes for the simulation

    // Fix positions for the nodes based on initial x, y values
    // dataset.nodes.forEach(d => {
    //     d.fx = d.x;
    //     d.fy = d.y;
    // });
    
    simulation.on("tick", ticked);
        
    simulation.force("link")
        .links(dataset.links);

    function ticked() {
        link
            .attr("x1", d => d.source.x)
            .attr("y1", d => d.source.y)
            .attr("x2", d => d.target.x)
            .attr("y2", d => d.target.y);
    
        node
            .attr("cx", d => d.x)
            .attr("cy", d => d.y);
        
        // Update text positions during the simulation
        nodeText
            .attr("dx", d => d.x + 25)
            .attr("dy", d => d.y + 5);
    }

    function updateNodeState() {
        // Fetch the current global state
        const currentState = window.globalState;
    
        if (currentState != 'error') {
            // Update the appearance of all nodes
            svg.selectAll("circle")
                .style("fill", function(d) {
                    return d.state === currentState ? "orange" : "blue";
                });
        }
    }
    
    setInterval(updateNodeState, 1000);

    
});