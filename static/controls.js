tailSimControls = Object();

tailSimControls.getValById = function (elID){
    return document.getElementById(elID).value;
}

tailSimControls.handleRebuildBtn = function(){
    const linkCount = +this.getValById("link_count") + 1;
    const tailLength = +this.getValById("tail_length")/100;
    const linkLength = tailLength/(linkCount-1);
    const linkWidth = +this.getValById("link_width") / 1000;
    const linkMass = +this.getValById("link_mass") / 1000.0;
    console.log([linkCount, linkLength, linkMass, linkWidth])
    TailSim.createLinks(linkCount, linkLength, linkWidth, linkMass);
}

//Hides all controls of the ff_hideable class except for the specified set of controls
//Used to hide/show different control sets for each forcing function
tailSimControls.hideAllControlsExcept = function (controlSetID, selector=".ff_hideable"){
    const controls = document.querySelectorAll(selector);
    for (let i = 0; i < controls.length; i++){
        if(!(controls[i].id === controlSetID)){
            controls[i].hidden = true;
        }else{
            controls[i].hidden = false;
        }
    }
}

tailSimControls.handleForcingFunctionChange = function(el,ev){
    const STATIC_CONTROLS = "controls_static";
    const TENSION_SWEEP_CONTROLS = "controls_tension_sweep";

    const forcingFunctionName = this.getValById("forcingFunctionInput");

    if (forcingFunctionName === STATIC_CONTROLS){
        console.log("static")
        this.hideAllControlsExcept(STATIC_CONTROLS);
        const staticTension = +this.getValById("staticTension");
        TailSim.setForcingFunctionStrat(TailSim.staticTensionStrat(staticTension));
    }

    else if(forcingFunctionName === TENSION_SWEEP_CONTROLS){
        this.hideAllControlsExcept(TENSION_SWEEP_CONTROLS)
        const sweepAmplitude = +this.getValById("tensionSweepAmplitude");
        const sweepFrequency = +this.getValById("tensionSweepFrequency");
        TailSim.setForcingFunctionStrat(TailSim.sweepTensionStrat(sweepAmplitude, sweepFrequency));
    }

    else {
        console.error("Forcing function name was not found: " + String(forcingFunctionName))
    }
}

tailSimControls.installForcingFunctionControlChangeEvents = function(){
    document.getElementById("forcingFunctionInput").addEventListener("change", (el,ev) => {tailSimControls.handleForcingFunctionChange(el,ev)})
    const inputs = document.querySelectorAll(".ff_hideable input");
    for (let i = 0; i < inputs.length; i++){
        inputs[i].addEventListener("change", tailSimControls.handleForcingFunctionChange.bind(this))
    }
}

tailSimControls.installAdvancedOptionEvents = function(){
    const angleConstraintID = "angleConstraint";
    const velDampingID = "velocityDamping";
    const vel2DampingID = "velSquaredDamping";

    document.getElementById(angleConstraintID).addEventListener("change", (ev)=>{
        const angleDegrees = +ev.target.value;
        const angleRadians = angleDegrees*(Math.PI/ 180);
        TailSim.bendLimit = angleRadians;
    })

    document.getElementById(velDampingID).addEventListener("change", (ev) => {
        const val = +ev.target.value;
        TailSim.velocityDamping = val;
    });

    document.getElementById(vel2DampingID).addEventListener("change", (ev) => {
        const val = +ev.target.value;
        TailSim.velocityDamping = val;
    });
}

tailSimControls.installStatsResetEvent = function(){
    document.getElementById("resetStats").addEventListener("click", (evt) => {
        TailSim.resetStats();
    });
}

tailSimControls.formatStatsNumber = function(num){
    return String((Math.round(num * 10)/10).toFixed(1))
}

tailSimControls.updateStats = function(){
    TailSim.updateStats();
    stats = TailSim.statistics;
    document.getElementById("cableDisplacement").textContent = tailSimControls.formatStatsNumber(stats.cableDisplacement * 1000) + "mm";
    document.getElementById("cableDisplacementMax").textContent = tailSimControls.formatStatsNumber(stats.cableDisplacementMax * 1000) + "mm";
    document.getElementById("horizontalDisplacement").textContent = tailSimControls.formatStatsNumber(stats.horizontalDisplacement * 1000) + "mm";
    document.getElementById("horizontalDisplacementMax").textContent = tailSimControls.formatStatsNumber(stats.horizontalDisplacementMax * 1000) + "mm";
}

window.addEventListener("load", (el, ev) => {
    //Handle reset button
    document.getElementById("update_simulation").addEventListener("click", () => {tailSimControls.handleRebuildBtn()});
    tailSimControls.handleRebuildBtn();
    //Handle forcing function change
    tailSimControls.installForcingFunctionControlChangeEvents();
    tailSimControls.handleForcingFunctionChange();
    //Handle advanced options
    tailSimControls.installAdvancedOptionEvents();
    //Handle reset stats button
    tailSimControls.installStatsResetEvent();
    //Statistics panel
    window.setInterval(tailSimControls.updateStats, 100);
    console.log("loaded.")
});