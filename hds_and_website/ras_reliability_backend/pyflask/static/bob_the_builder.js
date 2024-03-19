// Click event handler for the buttons
function handleClick(buttonLabel) {
    const Robotvisualisation = d3.select("#Robotvisualisation");
    const HDSvisualisation = d3.select("#HDSvisualisation");
  
    if (buttonLabel === "Robot") {
      // Toggle visibility of the visualisation SVG
      const isHidden = Robotvisualisation.style("display") === "none";
      Robotvisualisation.style("display", isHidden ? "block" : "none");
    } else if (buttonLabel === "HDS") {
      const isHidden = HDSvisualisation.style("display") === "none";
      HDSvisualisation.style("display", isHidden ? "block" : "none");
    }
}  