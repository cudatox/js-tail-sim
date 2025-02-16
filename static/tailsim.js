TailSim = Object();

TailSim.linkArray = [];
TailSim.tension = 0.0;
TailSim.velocityDamping = 0.2;
TailSim.velocitySquaredDamping = 0.0;
TailSim.bendLimit = Math.PI/4;

TailSim.inelasticAngleConstraint = false;
TailSim.distanceConstraintTolerance = 0.01;

TailSim.simTime = 0.0;

TailSim.forcingFunctionStrat = ()=>{};

TailSim.statistics = {
    "maxActuationDistance" : 0.0,
    "minActuationDistance" : 0.0,
};

TailSim.updateStats = function(){
    const actuationDistance = this.getRestingBowdenLength() - this.getBowdenLength();
    if (actuationDistance > this.statistics.maxActuationDistance){
        this.statistics.maxActuationDistance = actuationDistance
    }
    if (actuationDistance < this.statistics.minActuationDistance){
        this.statistics.minActuationDistance = actuationDistance
    }
}

TailSim.createLink = function() {
    return {
        "length" : 0.0,
        "width" : 0.0,
        "mass" : 0.0,
        "position" : [0.0, 0.0],
        "lastPosition" : [0.0, 0.0],
        "acceleration" : [0.0, 0.0],
        "forceVector" : [0.0, 0.0]
    };
};

TailSim.createLinks = function(n, length=65.0/1000.0, width=65.0/1000.0, mass=0.1){
    TailSim.linkArray = [];
    for (let i =0; i < n; i++){
        let link = TailSim.createLink();
        link.length = length;
        link.width = width;
        link.mass = mass;
        link.position = [0.0, -1 * i * length];
        link.lastPosition = [0.0, -1 * i * length];
        TailSim.linkArray.push(link);
    }
}

TailSim.VecSub = function (v1, v2){
    return [v1[0] - v2[0], v1[1] - v2[1]];
}

TailSim.VecAdd = function (v1, v2){
    return [v2[0] + v1[0], v2[1] + v1[1]];
}


TailSim.VecNorm = function (v){
    const mag = Math.sqrt(Math.pow(v[0], 2) + Math.pow(v[1],2));
    return [v[0] / mag, v[1] / mag];
}

//finds a vector perpendicular to A
TailSim.VecPerp = function (A){
    return [A[1], -1* A[0]];
}

TailSim.VecMulScalar = function(V, s){
    return ([V[0] * s, V[1] * s])
}

TailSim.VecDot = function(v1, v2){
    return v1[0] * v2[0] + v1[1] * v2[1];
}

//Vector cross product. Returns as a scalar as this is in 2D.
TailSim.VecCross = function(A, B){
    return A[0] * B[1] - A[1] * B[0];
}

TailSim.VecMag = function(A){
    return Math.sqrt(A[0]*A[0] + A[1]*A[1]);
}

//Projects A onto B
TailSim.VecProj = function(a, b){
    const normB = TailSim.VecNorm(b);
    return TailSim.VecMulScalar(normB, TailSim.VecDot(normB, a));
}

TailSim.isZeroVector = function(A){
    return (A[0] == 0 && A[1] == 0);
}

//Cosine of two vectors placed tail to tail
TailSim.VecCos = function(A, B){
    return this.VecDot(A, B) / (this.VecMag(A) * this.VecMag(B));
}

//Sine of two vectors placed tail to tail
TailSim.VecSin = function(A, B){
    return Math.abs(this.VecCross(A, B)) / (this.VecMag(A) * this.VecMag(B));
}

//Finds an angle between two vectors from 0 .. 2*PI
TailSim.VecAngle = function(A, B){
    const tanAngle = Math.atan2(this.VecCross(A, B), this.VecDot(A,B));
    if (tanAngle < 0){
        return tanAngle + 2 * Math.PI;
    }
    return tanAngle;
}

//Computes a vector perpendicular to the current link and two points on that vector representing the
//location that the control cable would pass though
TailSim.LinkDiskEyelets = function(linkIndex){
    const link = this.linkArray[linkIndex];

    //First, handle the special case of the first link.
    if (linkIndex == 0){
        return {
            "diskR" : [link.width/2, 0],
            "diskL" : [-link.width/2, 0],
            "vector" : [1,0]
        }
    }


    const prevLink = this.linkArray[linkIndex-1];

    //Find a normal vector perpendicular to the vector from this link to the previous one
    const normPerp = TailSim.VecNorm(TailSim.VecPerp(TailSim.VecSub(prevLink.position, link.position)));

    //Find the points representing the left and right side of the disk
    const diskR = TailSim.VecAdd(TailSim.VecMulScalar(normPerp, link.width/2), link.position);
    const diskL = TailSim.VecAdd(TailSim.VecMulScalar(normPerp, -link.width/2), link.position);

    return {
        "diskR" : diskR,
        "diskL" : diskL,
        "vector" : normPerp
    }

}

//Returns a function that transforms and scales world coordinates to screen space coordinates for drawing
TailSim.buildScreenSpaceTransform = (canvasElement) => {
    let totalLength = 0.0;
    const width = canvasElement.width;
    const height = canvasElement.height;

    for (let link of TailSim.linkArray){
        totalLength += link.length;
    }

    const scale = 0.8 * Math.min(width, height) / totalLength;
    const centerX = width / 2;

    return (vec) => {
        tx = vec[0] * scale + centerX;
        ty = vec[1] * scale * -1;
        return [tx,ty];
    }
}


TailSim.getRestingBowdenLength = function(){
    let sum = 0;
    for (let i = 1; i < TailSim.linkArray.length; i++){
        sum += TailSim.linkArray[i].length;
    }
    return sum;
}

TailSim.getBowdenLength = function(){
    let sum = 0;
    let prevEyelets = TailSim.LinkDiskEyelets(0);
    for (let i = 1; i < TailSim.linkArray.length; i++){
        const thisEyelet = TailSim.LinkDiskEyelets(i);
        sum += this.VecMag(this.VecSub(prevEyelets.diskR, thisEyelet.diskR));
        prevEyelets = TailSim.LinkDiskEyelets(i);
    }
    return sum;
}

TailSim.drawToCanvas = function(canvasElement, linkArray=TailSim.linkArray){

    let ctx = canvasElement.getContext("2d");

    //First link is always an anchor (doesn't move at all and defines the start of the tail)
    let anchorPos = linkArray[0].position;
    let lastPos = anchorPos;
    let sst = TailSim.buildScreenSpaceTransform(canvasElement);

    //Clear canvas
    ctx.clearRect(0,0,canvasElement.width, canvasElement.height);
    
    for (let i = 1; i < TailSim.linkArray.length; i++){
        const link = TailSim.linkArray[i];
        //Draw the spine
        ctx.beginPath();
        ctx.strokeStyle = "rgb(0 0 0)"
        ctx.moveTo(...sst(lastPos));
        ctx.lineTo(...sst(link.position));
        ctx.closePath();
        ctx.stroke();

        //Draw the point mass
        ctx.moveTo(...sst(link.position));
        ctx.beginPath();
        ctx.arc(...sst(link.position), 10, 0, 2 * Math.PI);
        ctx.closePath();
        ctx.fill();

        //Draw the "disk"
        //Find a normal vector perpendicular to the vector from this link to the previous one
        //Then, find the points representing the left and right side of the disk and draw a line
        //Between them.
        
        const linkEyelets = this.LinkDiskEyelets(i)
        ctx.beginPath();
        ctx.moveTo(...sst(linkEyelets.diskL));
        ctx.lineTo(...sst(linkEyelets.diskR));
        ctx.closePath();
        ctx.stroke();

        //Draw the force vectors
        ctx.beginPath();
        ctx.strokeStyle = "rgb(255 0 0)"
        ctx.moveTo(...sst(link.position));
        ctx.lineTo(...sst(this.VecAdd(link.position, this.VecMulScalar(link.forceVector, 0.1))));
        ctx.closePath();
        ctx.stroke();
        
        lastPos = link.position;
    }

}

//Moves link2 in the direction of link1 so that the distance between
//link1 and link2 is equal to the length of link2
TailSim.constrain = function(link1, link2){
    const dist = link2.length;
    const vecNorm = TailSim.VecNorm(TailSim.VecSub(link2.position, link1.position));
    const newPos = TailSim.VecAdd(TailSim.VecMulScalar(vecNorm, dist), link1.position);
    link2.position = newPos;
}


//Constrains the angle formed by the vector between this node and the previous node and the
//vector between this node and the next node by moving the next node
TailSim.angleConstrain = function(index, deltaAngle){
    const vector = this.VecSub(this.linkArray[index+1].position, this.linkArray[index].position);
    let prevVector = [0.0,0.0]
    
    if (index == 0){
        //Account for the anchor link by making a virtual point above it and computing a vector to it.
        prevVector = this.VecSub([0, this.linkArray[0].position[1] - this.linkArray[0].length], this.linkArray[index].position);
    }else{
        prevVector = this.VecSub(this.linkArray[index-1].position, this.linkArray[index].position);
    }

    const angle = this.VecAngle(vector, prevVector);
    let constrainedAngle = angle;

    if (angle < (Math.PI - deltaAngle)){
        constrainedAngle = Math.PI + deltaAngle;
    }else if (angle > (Math.PI + deltaAngle)){
        constrainedAngle = Math.PI - deltaAngle;
    }else{
        //No constraint needed.
        return;
    }

    let prevAngle = Math.atan2(prevVector[1], prevVector[0]);
    if (prevAngle < 0){prevAngle += 2 * Math.PI;}


    //Find the new vector, if the constrained angle differs
    if(angle != constrainedAngle){
        //Add the rotation of the previous vector 
        let effectiveAngle = constrainedAngle + prevAngle;
        const newX = Math.cos(effectiveAngle) * this.linkArray[index+1].length + this.linkArray[index].position[0];
        const newY = Math.sin(effectiveAngle) * this.linkArray[index+1].length + this.linkArray[index].position[1];
        const newPos = [newX, newY];
        this.linkArray[index+1].position = newPos;

        if (this.inelasticAngleConstraint){
            //If this constraint should not preserve momentum, kill all momentum 
            //by moving the last position to the new position. (Improves stability at the cost of accuracy)
            this.linkArray[index+1].lastPosition = newPos;
        }
    }


}

TailSim.distanceConstraintsResolved = function(){
    for (let i = 1; i < this.linkArray.length-1; i++){
        const dist = this.VecMag(this.VecSub(this.linkArray[i], this.linkArray[i-1]));
        if (Math.abs(this.linkArray[i].length - dist) > this.distanceConstraintTolerance){
            return false;
        }
    }
    return true;
}

TailSim.FABRIKConstrian = function(){

    TailSim.linkArray[0].position = [0,0]

    for (let relaxCount = 0; relaxCount < 10; relaxCount++){

        //Move backwards, puling the links away from the anchor
        for (let i = TailSim.linkArray.length-1; i > 1; i--){
            TailSim.constrain(TailSim.linkArray[i], TailSim.linkArray[i-1]);
        }

        //Move forward through the array, pulling the links onto the anchor
        for (let i = 0; i < TailSim.linkArray.length - 1; i++){
            TailSim.constrain(TailSim.linkArray[i], TailSim.linkArray[i+1]);
        }

        if (this.distanceConstraintsResolved()){
            break;
        }
    }

    for (let i = TailSim.linkArray.length - 2; i >= 1; i--){
        this.angleConstrain(i, this.bendLimit);
    }
    
}

TailSim.verlet = function(timeStep){
    for (let i = 1; i < TailSim.linkArray.length; i++){
        //Compute the effect of the acceleration, a * t ^ 2
        const link = TailSim.linkArray[i];
        const accelTerm = TailSim.VecMulScalar(link.acceleration, timeStep * timeStep);
        const lastPosition = link.lastPosition;
        const currentPosition = link.position;
        const newPosition = TailSim.VecAdd(TailSim.VecSub(TailSim.VecMulScalar(currentPosition, 2), lastPosition), accelTerm);
        link.lastPosition = link.position;
        link.position = newPosition;
    }
}

TailSim.clearAcceleraton = function(){
    for (let i = 1; i < TailSim.linkArray.length; i++){
        const link = TailSim.linkArray[i];
        link.acceleration = [0, 0];
    }
}

TailSim.applyGravity = function(){
    //Apply gravity.
    for (let i = 1; i < TailSim.linkArray.length; i++){
        const link = TailSim.linkArray[i];
        link.acceleration = TailSim.VecAdd([0, -9.8], link.acceleration);
    }
}

TailSim.applyLastLinkTension = function(){
    const lastLinkIndex = TailSim.linkArray.length-1;
    const lastLink = TailSim.linkArray[lastLinkIndex];
    const linkDiskEyelets = this.LinkDiskEyelets(lastLinkIndex);
    const prevLinkDiskEyelets = this.LinkDiskEyelets(lastLinkIndex - 1);

    let tensionVector = [0,0];
    let leverVector = [0,0];

    if (this.tension > 0){
        tensionVector = this.VecMulScalar(this.VecNorm(this.VecSub(prevLinkDiskEyelets.diskR, linkDiskEyelets.diskR)), this.tension);
        leverVector = this.VecSub(prevLinkDiskEyelets.diskR, lastLink.position);
    }else{
        tensionVector = this.VecMulScalar(this.VecNorm(this.VecSub(prevLinkDiskEyelets.diskL, linkDiskEyelets.diskL)), this.tension * -1);
        leverVector = this.VecSub(prevLinkDiskEyelets.diskL, lastLink.position);
    }

    const torque = this.VecCross(leverVector, tensionVector);
    const radialForce = torque / this.VecMag(leverVector);
    const radialAcceleration = radialForce / lastLink.mass;
    const accelVec = this.VecMulScalar(linkDiskEyelets.vector, radialAcceleration);

    lastLink.forceVector = this.VecMulScalar(linkDiskEyelets.vector, radialForce);
    lastLink.acceleration = TailSim.VecAdd(accelVec, lastLink.acceleration);
}

//Computes the force applied to the links by the control cable under tension.
//The control cable is able to move freely through the eyelets of the spine disks,
//So it can only impose a radial (outward) force on the tail.
TailSim.computeRadialTensionForce = function(index){
    //TODO: it would be very cool to draw these vectors.
    const previousLinkDiskEyelets = this.LinkDiskEyelets(index-1);
    const linkDiskEyelets = this.LinkDiskEyelets(index);
    const nextLinkDiskEyelets = this.LinkDiskEyelets(index+1);

    const radialNormal = linkDiskEyelets.vector;
    let prevTensionVector = [0.0, 0.0];
    let nextTensionVector = [0.0, 0.0];

    if (this.tension < 0.0){
        prevTensionVector = this.VecMulScalar(this.VecNorm(this.VecSub(previousLinkDiskEyelets.diskL, linkDiskEyelets.diskL)), this.tension);
        nextTensionVector = this.VecMulScalar(this.VecNorm(this.VecSub(nextLinkDiskEyelets.diskL, linkDiskEyelets.diskL)), this.tension);   
    }else{
        prevTensionVector = this.VecMulScalar(this.VecNorm(this.VecSub(previousLinkDiskEyelets.diskR, linkDiskEyelets.diskR)), this.tension);
        nextTensionVector = this.VecMulScalar(this.VecNorm(this.VecSub(nextLinkDiskEyelets.diskR, linkDiskEyelets.diskR)), this.tension);
    }
    
    let prevForce = [0,0];
    let nextForce = [0,0];

    //Cannot project onto a zero vector.
    if (!this.isZeroVector(radialNormal)){
        prevForce = this.VecProj(prevTensionVector, radialNormal);
        nextForce = this.VecProj(nextTensionVector, radialNormal);
    }   

    //Make sure the forces are in the correct direction if tension is negative.
    if (this.tension < 0.0){
        prevForce = this.VecMulScalar(prevForce, -1.0);
        nextForce = this.VecMulScalar(nextForce, -1.0);
    }

    return this.VecAdd(prevForce, nextForce)
}

TailSim.applyTension = function(){
    this.applyLastLinkTension();

    //TODO: handle case where lastLink is linkArray.length - 2??
    for (let i = 1; i < this.linkArray.length - 1; i++){
        const link = this.linkArray[i];
        const radialForce = this.computeRadialTensionForce(i);
        const radialAcceleration = this.VecMulScalar(radialForce, 1/link.mass);
        link.acceleration = this.VecAdd(link.acceleration, radialAcceleration);
        link.forceVector = this.VecMulScalar(radialForce, 0.5);
    }

}

//This function applies a dampening force proportional to and in a direction opposite of the velocity
//of the link particle. This aids in system stability and results in faster convergence.
TailSim.applyVelocityDampening = function(timeStep) {
    for (let i = 1; i <= this.linkArray.length - 1; i++){
        const link = this.linkArray[i];
        const velocity = this.VecMulScalar(this.VecSub(link.position, link.lastPosition), 1/timeStep);
        const dampening = this.VecMulScalar(velocity, this.velocityDamping * 1/link.mass * -1);
        link.acceleration = this.VecAdd(link.acceleration, dampening);

        if (!this.isZeroVector(velocity)){
            const velocitySquared = this.VecDot(velocity, velocity);
            const v2dampening = this.VecMulScalar(this.VecNorm(velocity), velocitySquared * this.velocitySquaredDamping * 1/link.mass * -1);
            link.acceleration = this.VecAdd(link.acceleration, v2dampening);
        }
        
    }
}

TailSim.updateAcceleration = function(timeStep){
    this.clearAcceleraton();
    this.applyGravity();
    this.applyTension();
    this.applyVelocityDampening(timeStep);
}

TailSim.stepSimulation = function(timeStep){
    TailSim.updateAcceleration(timeStep);
    TailSim.verlet(timeStep);
    TailSim.FABRIKConstrian();
}


TailSim.setForcingFunctionStrat = function(ff_strategy){
    this.forcingFunctionStrat = ff_strategy;
}

TailSim.staticTensionStrat = function(tension){
    return () => {
        return tension;
    }
}

TailSim.sweepTensionStrat = function(tension, freq){
    return () => {
        return Math.sin((TailSim.simTime/1000) * (freq * 2 * Math.PI)) * tension;
    }
}

//This needs force(torque) falloff and it needs to sloooooowly move to position as a servo would
TailSim.constantRateSpringStrat = function(springForce, maxRetraction){
    //if retraction < maxRetraction then tension = springForce else tension = -springForce
    return () => {
        const restLength = this.getRestingBowdenLength();
        const currentLength = this.getBowdenLength();
        const delta = -currentLength + restLength;
        console.log(delta)
        if (maxRetraction > 0){
            if (delta < maxRetraction && delta > 0){
                return springForce * delta/maxRetraction;
            }else{
                return 0;
            }
        }else{
            if (delta > maxRetraction && delta < 0){
                return -springForce;
            }else{
                return 0;
            }
        }

    }
}

function animate(){
    for (var i =0; i < 2; i++){
        TailSim.stepSimulation(1.0/120);
        TailSim.simTime += 1000.0/120;
        TailSim.tension = TailSim.forcingFunctionStrat();
    }
    TailSim.drawToCanvas(document.getElementById("sim_canvas"))
}

function testDraw(){
    TailSim.createLinks(7);
    TailSim.forcingFunctionStrat = TailSim.sweepTensionStrat(2.0, 0.5);
    window.setInterval(animate, 1000/60);
}

window.addEventListener('load', testDraw);