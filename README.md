### FLOW FIELD

# Grid based pathfinding based on node flow.

## Create the flow field base object
```javascript
let flowGrid = new flow_grid();
```

## Generate flow for at least one goal node
```javascript
flowGrid.generateFlowField(goalCell);
```