//find in line 14 of Front_susp.js (brown)
//for bellcrank p1 bell crank mount point, p2 pushrod mount, p3 shock mount point
const bellcrank = [[0,0,0],[0,0,.03],[0,.03,0]]
//for chassis, p1 bell crank mount point
//for chassis, p1 lower a p1, p2 lower a, p3 upper a p1, p4 upper a p2, p5 tie rod conn, p6 bellcrank mount
const chassis = [[0,.2,-.09],[.3,.2,-.09],[0,0.2,.09],[.3,0.2,.09],[-.05,0.22,-.09], [0,0,0.18]]
const pushrodlength = 0.38

//683
function Bellcrank(bellcrank,chassis){
  //local coordinate representations of each link
  this.bellcrank = bellcrank
  this.chassis = chassis


//find in line 691 of Front_susp.js (brown)
  //now create global coordinates for each body
  //format [X,Y,Z,r,p,y] for each body
  var chassisGlobal= [0,0,0,0,0,0]
  var bellcrankGlobal = [chassisGlobal[0],chassisGlobal[1],chassisGlobal[2],0,0,0]

  //print(lowerAGlobal)
  this.bellcrankGlobal = bellcrankGlobal
  this.chassisGlobal= chassisGlobal
  
  this.getqArrayFromGlobals = function(){
    // print(this.lowerAGlobal)
    var q = [this.bellcrankGlobal[3],this.bellcrankGlobal[4],this.bellcrankGlobal[5],this.chassisGlobal[3],this.chassisGlobal[4],this.chassisGlobal[5]]
    return q
  }

  this.updateGlobalPositionsFromQ = function(q){
   //q are: [bellr bellp bella chassisr chassisp chassisa]
    this.bellcrankGlobal = [this.bellcrankGlobal[0],this.bellcrankGlobal[1],this.bellcrankGlobal[2],q[0],q[1],q[2]]
    this.chassisGlobal = [this.chassisGlobal[0],this.chassisGlobal[1],this.chassisGlobal[2],q[3],q[4],q[5]]
   
  }



  //https://www.johndcook.com/blog/2018/05/05/svd/ <-use SVD to get Moore-Penrose.
  this.shultzInverse = function(M){
    iM = math.eye(M._size[1])
    for(let k=0;k<10;k++){
      iM = math.multiply(iM,math.subtract(math.multiply(math.eye(M._size[0]),2),math.multiply(M,iM))) 
    }
    return iM
  }

  //makes a rotation matrix
  // http://web.mit.edu/2.05/www/Handout/HO2.PDF
  this.makeRotation = function(roll, pitch, yaw){
    const R = math.matrix([[cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll),cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll)],[sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll)],[-sin(pitch),cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
    return R
  }

  //this function calculates global position given local. global origin in form [xyzrpa]
  this.calcGlobal = function(origin, qlocal){
    pglobal = []
    //create tall vector for rotation operation
    localvec = math.matrix([[qlocal[0]],[qlocal[1]],[qlocal[2]]])
    // if(this.debug){print("localvec: "+str(localvec))}
    //create rotation matrix
    R = this.makeRotation(origin[3],origin[4],origin[5])
    // if(this.debug){print("roll: "+str(origin[3])+", pitch: "+str(origin[4])+", yaw:"+str(origin[5]))}
    // if(this.debug){print("R: "+str(R))}
    //rotate point
    rotvec = math.multiply(R,localvec)
    // if(this.debug){print("localvec: "+str(rotvec))}
    //create plain array with translation by body origin
    pglobal = [origin[0]+rotvec.subset(math.index(0,0)),origin[1]+rotvec.subset(math.index(1,0)),origin[2]+rotvec.subset(math.index(2,0))]

    //return global position of point
    return pglobal
  }

  //loops through an array of 3-arrays, converting local coordinates into global coordinates of each point.
  this.getDrawPoints = function(origin,plocal){
    var pglobal = [];
    for(let k = 0;k<plocal.length;k++){
      //pull out this local xyz array
      thislocal = plocal[k]
      //now pass into the local to global function
      thisglobal = this.calcGlobal(origin,plocal[k])
      pglobal.push(thisglobal)
    }
    return pglobal
  }

  //constraint in X direction
  this.calcConstraints = function(q){
    // actual physical constraints are:
    // spherical chassis p1 to lower A p1
    // sperical chassis p2 to lower A p2
    // spherical chassis p3 to upper A p1
    // spherical chassis p4 to upper A p2
    // spherical lower A p3 to upright p1
    // spherical upper A p3 to upright p2
    // distance  upright p3 to chassis p5

    //idea to try: make 13 gen coords (just include dummy gen coords for tie rod loc?)

  

    //q are: [bellr bellp bella chassisr chassisp chassisa]
    //known coords: bellx,belly,bellz, chassisx, chassisy, chassisz ()
    // write 3 eqn for revolute bellcrank to chassis

    //using the current q, get local origin (global) variables for each link
    //this allows us to use the nice functions we have to calculate constraints using a 'local' q
    //which is required for calculating numerical Jacobian, for example
    bellcrankGlobal = [this.bellcrankGlobal[0], this.bellcrankGlobal[1], this.bellcrankGlobal[2], q[0],q[1],q[2]]
    chassisGlobal = [this.chassisGlobal[0], this.chassisGlobal[1], this.chassisAGlobal[2], q[3],q[4],q[5]]


    //calculate phi1-phi3 (rev)
    //global position of bellcrank (rev1)
    rev1_bellglobal = this.calcGlobal(bellcrankGlobal,this.bellcrank[1])
    rev1_chassisglobal = this.calcGlobal(this.chassisGlobal,this.chassis[1])
    //actually write the three constraint eqns for this spherical
    phi1 = rev1_bellglobal[0]-rev1_chassisglobal[0]
    phi2 = rev1_bellglobal[1]-rev1_chassisglobal[1]
    phi3 = rev1_bellglobal[2]-rev1_chassisglobal[2]


    // bellr = [[1, 0, 0],[0, 1, 0],[0, 0, 1]]        
    //  bellp = [[1, 0, 0],[0, 1, 0],[0, 0, 1]]       
    // bella = [[1, 0, 0],[0, 1, 0],[0, 0, 1]]         
    // Rbell = bellr*bellp*bella
   
    //  phi4 = 
   
 
    //this not done
    //calculate distance (double spherical) D1
    //this constraint tells us that the distance between LBJ and UBJ (between)
    d_bell_push = this.calcGlobal(this.belcrankGlobal,this.bellcrank[2])
    d_a_push = this.calcGlobal(this.lowerAGlobal,this.lowerA[3])
    phi13 = (math.pow(d_bell_push[0]-d_a_push[0],2)+math.pow(d_bell_push[1]-d_a_push[1],2)+math.pow(d_bell_push[2]-d_a_push[2],2)) - math.pow(this.pushrodlength,2)


    // //calculate dot product between upright X and global X to keep it from flipping wildly.
    // uprightAxisGlobal = this.calcGlobal([0,0,0,this.uprightGlobal[3],this.uprightGlobal[4],this.uprightGlobal[5]],[1,0,0])
    // phi14 = uprightAxisGlobal[0] - 1

    constraints = math.matrix([[phi1],[phi2],[phi3],[phi4],[phi5],[phi6],[phi7],[phi8],[phi9],[phi10],[phi11],[phi12],[phi13]])

    // var fx = this.r2*cos(this.t2)+this.r3*cos(math.subset(q,math.index(0,0))) - this.r4*cos(math.subset(q,math.index(1,0))) - this.r1*cos(this.t1)
    // var fy = this.r2*sin(this.t2)+this.r3*sin(math.subset(q,math.index(0,0))) - this.r4*sin(math.subset(q,math.index(1,0))) - this.r1*sin(this.t1)
    // var constraints = math.matrix([[fx],[fy]])
    // print(str(constraints))
    return constraints
  }
