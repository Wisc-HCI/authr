import { Injectable, Optional }      from '@angular/core';
import { Router }                    from '@angular/router';
import { Observable, from, of }      from 'rxjs';
import { first }                     from 'rxjs/operators';
import * as ROS3D                    from 'ros3d';
import * as ROSLIB                   from 'roslib';
import { MatSnackBar }               from '@angular/material';
import * as THREE                    from 'three-full';
import { Plan }                      from '../models/plan';
import { Task }                      from '../models/task';
import { Agent, AgentType }          from '../models/agent';
import { Thing, ThingType }          from '../models/thing';
import { Destination }               from '../models/destination';
import { Cartesian3D }               from '../models/cartesian-3d';
import { EulerAngle }                from '../models/eulerAngle';
import { Therblig }                  from '../models/therblig';
import { TherbligPrimitive }         from '../models/therbligPrimitive'
import { saveAs }                    from 'file-saver';
import { DragulaService }            from 'ng2-dragula';
import { Prefix }                         from './mesh-url-prefix.service';

@Injectable({
    providedIn: 'root',
})

export class AuthrServiceConfig {
}

@Injectable()
export class AuthrService {

    //==========================================================================
    // Cache Implementation
    //==========================================================================

    public _plan: any = {
      name: '',
      version: '',
      timeweight: 0,
      costweight: 0,
      task_keys: [],
      tasks: {},
      macros: {},
      therbligs: {},
      agents: {},
      things: {},
      destinations: {},
      tofs: {},
      tof_table: {}
    };
    public _fixedFrame: string = 'undefined';
    public _robot: string = 'undefined';
    public _counts: any = {
      agents: 0,
      robots: 0,
      humans: 0,
      things: 0,
      tasks: 0,
      destinations: 0,
      therbligs: 0,
      macros: 0
    };
    public _primitives: any = {};
    public _primitiveKeys: string[] = []

    //==========================================================================
    // Initialization
    //==========================================================================

    constructor(public snackbar: MatSnackBar,
                public router: Router,
                public dragulaService: DragulaService) {}

    public address: string = 'localhost:9090';
    public ros: ROSLIB.Ros;

    // ROS Clients
    public tfClient: ROSLIB.TFClient;
    public urdfClient: ROS3D.UrdfClient;
    public markerclient: ROS3D.MarkerArrayClient;
    public imclient_elements: ROS3D.InteractiveMarkerClient;
    public vizClient_elements: ROS3D.MarkerArrayClient;
    public tableMarker: ROS3D.MarkerClient;

    // ROS Services
    public getTOFService: ROSLIB.Service;
    public getRobotInfoService: ROSLIB.Service;
    public getPoseService: ROSLIB.Service;
    public getOptimizedPlanService: ROSLIB.Service;
    public setForceControlService: ROSLIB.Service;
    public getRequestService: ROSLIB.Service;
    public setRequestService: ROSLIB.Service;
    public createRequestService: ROSLIB.Service;
    public deleteRequestService: ROSLIB.Service;

    // ROS Topics
    public getCacheTopic: ROSLIB.Topic;
    public requestCacheRefresh: ROSLIB.Topic;
    public environmentRefreshTopic: ROSLIB.Topic;
    public interactiveMarkerRefreshTopic: ROSLIB.Topic;
    public simpleMarkerRefreshTopic: ROSLIB.Topic;
    public visualizationMarkerRefreshTopic: ROSLIB.Topic;
    public markerVisibilityTopic: ROSLIB.Topic;

    // ROS Actions
    public simulateClient: ROSLIB.ActionClient;

    // Utility Fields
    public _connected:boolean = false;
    public _loading:boolean = false;
    public _focused:string = null;
    public _dragging:string = null;
    public _invisible:string = null;
    public _redirectUrl:string = null;
    public _debugging:boolean = false;
    public _secure:boolean = false;
    public _split:number = 0;

    // Other Values

    public get debugging(): boolean {
      return this._debugging;
    }

    public get focused(): string {
      return this._focused;
    }

    public get invisible(): string {
      return this._invisible;
    }

    public get redirectUrl(): string {
      return this._redirectUrl;
    }

    public set focused(id:string) {
      this._focused = id;
    }

    public set invisible(id:string) {
      this._invisible = id;
    }

    public set debugging(value:boolean) {
      this._debugging = value;
    }

    public set redirectUrl(url:string) {
      this._redirectUrl = url;
    }

    public get connected(): boolean {
        return this._connected;
    }

    public get loading(): boolean {
        return this._loading;
    }

    public set loading(value:boolean) {
        this._loading = value;
    }

    public get secure(): boolean {
        return this._secure;
    }

    public set secure(value:boolean) {
        this._secure = value;
    }

    public get environment_split(): number {
        return this._split;
    }

    public set environment_split(value:number) {
        this._split = value;
    }

    public get workspace_split(): number {
        return 100-this._split;
    }

    public set workspace_split(value:number) {
        this._split = 100-value;
    }

    public emitCustomError(message:string) {
        this.snackbar.open(message,null, {duration: 5000})
    }

    public initializeOnConnection() {
        this._connected = true;
        this.setupServices();
        this.setupTopics();
        this.requestCacheRefresh.publish({data:'{}'});
        try {
          this.setupDragula();
        } catch {}

    }

    public connect(url: string): Observable<any> {

        this.address = url;
        if (this.secure) {
          this.ros = new ROSLIB.Ros({ url: "wss://" + this.address, groovyCompatibility: false });
        } else {
          this.ros = new ROSLIB.Ros({ url: "ws://" + this.address, groovyCompatibility: false });
        }

        return from(new Promise((resolve, reject) => {
          // Setup Connection Work
          this.ros.on('connection', () => {
              this.initializeOnConnection();
              resolve(true)
          });

          this.ros.on('error', (error) => {
              this._connected = false;
              this.router.navigate(['/login'])
              this.snackbar
              .open("Could not connect to ROS at "+this.address, null, {duration: 2000,});
              reject(error)
          });

          this.ros.on('close', () => {
              this._connected = false;
              this.router.navigate(['/login']);
              // this.snackbar.open("Connection closed to ROS at "+this.address, null, {duration: 2000});
              resolve(false)
          });

        }));

    };

    //==========================================================================
    // Setup Functions
    //==========================================================================

    public setupServices() {
      //TODO create a set_plan and clear_plan service

      // Set up services
      this.getTOFService = new ROSLIB.Service({
          ros : this.ros,
          name : '/authr_sim/get_tof',
          serviceType : 'authr/TimeOfFlightQuery'
      });

      this.getRequestService = new ROSLIB.Service({
          ros : this.ros,
          name : '/authr_plan/get_element',
          serviceType : 'authr/GetRequest'
      });

      this.setRequestService = new ROSLIB.Service({
          ros : this.ros,
          name : '/authr_plan/set_element',
          serviceType : 'authr/SetRequest'
      });

      this.createRequestService = new ROSLIB.Service({
          ros : this.ros,
          name : '/authr_plan/create_element',
          serviceType : 'authr/CreateRequest'
      });

      this.deleteRequestService = new ROSLIB.Service({
          ros : this.ros,
          name : '/authr_plan/delete_element',
          serviceType : 'authr/DeleteRequest'
      });

    }

    public updateFromCache(dataStr) {
      try {
        let cache = JSON.parse(dataStr);
        this._plan = cache.plan || {};
        this._fixedFrame = cache.fixed_frame || "Undefined";
        this._robot = cache.robot || "Undefined";
        this._counts = cache.counts || {};
        this._primitives = cache.primitives || {};
        this._primitiveKeys = cache.primitive_keys || []
      } catch {/* No update */
        console.log("Failed to fetch cache from backend.")
      }
    }

    public setupTopics() {
      this.getCacheTopic = new ROSLIB.Topic({
        ros: this.ros,
        name: '/authr_plan/get_cache',
        messageType: 'std_msgs/String'
      });

      this.getCacheTopic.subscribe(message => {
        this.updateFromCache(message.data);
      });

      this.requestCacheRefresh = new ROSLIB.Topic({
        ros: this.ros,
        name: '/authr_plan/request_cache_refresh',
        messageType: 'std_msgs/String'
      });

      // Send a request to load the plan
      // setTimeout(()=>{
      //   console.log("Requesting initial update from backend.")
      //   this.requestCacheRefresh.publish({data:'{}'});
      // },200)

      this.environmentRefreshTopic = new ROSLIB.Topic({
        ros: this.ros,
        name: '/authr_env/refresh',
        messageType: 'std_msgs/String'
      });

      this.interactiveMarkerRefreshTopic = new ROSLIB.Topic({
        ros: this.ros,
        name: '/interactive_marker/elements/refresh',
        messageType: 'std_msgs/String'
      });

      this.visualizationMarkerRefreshTopic =  new ROSLIB.Topic({
        ros: this.ros,
        name: '/visualization_marker/elements/refresh',
        messageType: 'std_msgs/String'
      });

      this.simpleMarkerRefreshTopic = new ROSLIB.Topic({
        ros: this.ros,
        name: '/authr_sim/simple_markers/refresh',
        messageType: 'std_msgs/String'
      });

      this.markerVisibilityTopic = new ROSLIB.Topic({
        ros: this.ros,
        name: '/authr_plan/marker_visibility',
        messageType: 'std_msgs/String'
      });
    }

    public setupClients(viewer:ROS3D.Viewer,createMarkers:boolean): Observable<boolean> {

      // always request fixedFrame to get most up to date data
      return from(this.getFixedFrameService().then(fixedFrame => {
        this._fixedFrame = fixedFrame;

        this.tfClient = new ROSLIB.TFClient({
            ros: this.ros,
            angularThres: 0.01,
            transThres: 0.01,
            rate: 10.0,
            fixedFrame: fixedFrame
        });

        this.urdfClient = new ROS3D.UrdfClient({
            ros: this.ros,
            tfClient: this.tfClient,
            path: Prefix.value + "/assets/meshes",
            rootObject: viewer.scene,
            loader: THREE.ColladaLoader
        });

        if (createMarkers) {
            this.imclient_elements = new ROS3D.InteractiveMarkerClient({
              ros : this.ros,
              tfClient : this.tfClient,
              path: Prefix.value + "/assets/meshes",
              topic : '/interactive_marker/elements',
              camera : viewer.camera,
              rootObject : viewer.selectableObjects
            });

            this.vizClient_elements = new ROS3D.MarkerArrayClient({
              ros: this.ros,
              tfClient: this.tfClient,
              path: Prefix.value + "/assets/meshes",
              topic: "/visualization_marker/elements",
              camera: viewer.camera,
              rootObject: viewer.selectableObjects
            });

        }

        this.markerclient = new ROS3D.MarkerArrayClient({
          ros : this.ros,
          tfClient : this.tfClient,
          path: Prefix.value + "/assets/meshes",
          topic : '/authr_sim/simple_markers/',
          camera : viewer.camera,
          rootObject : viewer.selectableObjects
        });

        this.tableMarker = new ROS3D.MarkerClient({
          ros: this.ros,
          tfClient: this.tfClient,
          topic: '/authr_env/table_marker',
          rootObject: viewer.scene
        });

        this.simulateClient = new ROSLIB.ActionClient({
            ros: this.ros,
            serverName : '/authr_sim/simulate',
            actionName : 'authr/SimulateAction'
        });

        return true;
      }));
    }

    public setupDragula() {
        this.dragulaService.createGroup('THERBLIG', {
            copy: (el, source) => {
                return source.id === 'sidebar';
            },
            copyItem: (type: string) => {
                return type;
            },
            accepts: (el, target, source, sibling) => {
                return target.id !== 'sidebar';
            }
        });
        this.dragulaService.createGroup("TASK", {
            direction: 'horizontal',
            moves: (el, source, handle) => {
                return handle.className == "task-handle mat-card-header" || handle.className == "mat-card-title"
            }
        });
    }

    public teardownClients() {
      this.tableMarker = undefined;
      this.vizClient_elements = undefined;
      this.simulateClient = undefined;
      this.markerclient = undefined;
      this.urdfClient = undefined;
      this.tfClient = undefined;
    }

    public callService(service, request): Promise<any> {
      return new Promise((resolve) => {
        service.callService(request, (result) => {
          resolve(result);
        });
      });
    }

    //==========================================================================
    // General
    //==========================================================================

    public get fixedFrame() {
      return this._fixedFrame;
    }

    public async getFixedFrameService() {
      var request = new ROSLIB.ServiceRequest({
        'type': 'fixed_frame'
      });

      return this.callService(this.getRequestService,request).then(result => {
        return result.response;
      });
    }

    public set fixedFrame(value:string) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'fixed_frame',
        'settings': value
      });

      this.callService(this.setRequestService,request);
    }

    public get robot() {
      return this._robot;
    }

    public get timeweight() {
      return this._plan.time_weight;
    }

    public set timeweight(value:number) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'time_weight',
        'settings': `${value}`
      });

      this.callService(this.setRequestService,request);
    }

    public get costweight() {
      return this._plan.cost_weight;
    }

    public set costweight(value:number) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'cost_weight',
        'settings': `${value}`
      });

      this.callService(this.setRequestService,request);
    }

    public get name() {
      return this._plan.name;
    }

    public set name(value:string) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'name',
        'settings': value
      });

      this.callService(this.setRequestService,request);
    }

    public get counts() {
      return this._counts;
    }

    public get tofs() {
      return this._plan.tofs;
    }

    public get tof_table() {
      return this._plan.tof_table;
    }

    public get primitives() {
      return this._primitives;
    }

    public get primitiveKeys() {
      return this._primitiveKeys;
    }

    public set primitiveKeys(keys) {
      // Disregard
    }

    public refreshTof(start_destination_id,end_destination_id) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'tof',
        'id': start_destination_id + '_' + end_destination_id,
        'settings': JSON.stringify({
          'start_destination_id': start_destination_id,
          'end_destination_id': end_destination_id
        })
      });

      this.callService(this.setRequestService,request);
    }

    //==========================================================================
    // Agents
    //==========================================================================

    public getAgentById(id) {
      return this._plan.agents[id];
    }

    public setAgentById(value, id) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'agent',
        'settings': JSON.stringify(value),
        'id': id
      });

      this.callService(this.setRequestService,request);
    }

    public get agentKeys() {
      return Object.keys(this._plan.agents).sort((a, b) => {return this._plan.agents[a].timestamp - this._plan.agents[b].timestamp});
    }

    public set agentKeys(value:any) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'agent_keys',
        'settings': value
      });

      this.callService(this.setRequestService,request);
    }

    public get agents() {
      return this._plan.agents;
    }

    public set agents(value:any) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'agent',
        'settings': value
      });

      this.callService(this.setRequestService,request);
    }

    //==========================================================================
    // Things
    //==========================================================================

    public getThingById(id) {
      return this._plan.things[id];
    }

    public setThingById(value, id) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'thing',
        'settings': JSON.stringify(value),
        'id': id
      });

      this.callService(this.setRequestService,request);
    }

    public get thingKeys() {
      return Object.keys(this._plan.things).sort((a, b) => {return this._plan.things[a].timestamp - this._plan.things[b].timestamp});
    }

    public set thingKeys(value:any) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'thing_keys',
        'settings': value
      });

      this.callService(this.setRequestService,request);
    }

    public get things() {
      return this._plan.things;
    }

    public set things(value:any) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'thing',
        'settings': value
      });

      this.callService(this.setRequestService,request);
    }

    //==========================================================================
    // Destinations
    //==========================================================================

    public getDestinationById(id) {
      return this._plan.destinations[id];
    }

    public setDestinationById(value, id) {
      console.log(value,id);
      var request = new ROSLIB.ServiceRequest({
        'type': 'destination',
        'settings': JSON.stringify(value),
        'id': id
      });

      this.callService(this.setRequestService,request);
    }

    public get destinationKeys() {
      return Object.keys(this._plan.destinations).sort((a, b) => {return this._plan.destinations[a].timestamp - this._plan.destinations[b].timestamp});
    }

    public set destinationKeys(value:any) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'destination_keys',
        'settings': value
      });

      this.callService(this.setRequestService,request);
    }

    public get destinations() {
      return this._plan.destinations;
    }

    public set destinations(value:any) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'destination',
        'settings': value
      });

      this.callService(this.setRequestService,request);
    }

    public validatePose(destination_id) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'validate_pose',
        'id': destination_id,
        'settings': '{}'
      });

      this.callService(this.setRequestService,request);
    }

    //==========================================================================
    // Tasks
    //==========================================================================

    public getTaskById(id) {
      return this._plan.tasks[id];
    }

    public setTaskById(value, id) {
      //this._plan.tasks[id] = {...value};
      var request = new ROSLIB.ServiceRequest({
        'type': 'task',
        'settings': JSON.stringify(value),
        'id': id
      });

      this.callService(this.setRequestService,request);
    }

    public get taskKeys() {
      return this._plan.task_keys;
    }

    public set taskKeys(value:any) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'task_keys',
        'settings': JSON.stringify(value)
      });

      this.callService(this.setRequestService,request);
    }

    public get tasks() {
      return this._plan.tasks;
    }

    public set tasks(value:any) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'task',
        'settings': JSON.stringify(value)
      });

      this.callService(this.setRequestService,request);
    }

    //==========================================================================
    // Therbligs
    //==========================================================================

    public getTherbligById(id:string) {
      return this._plan.therbligs[id];
    }

    public setTherbligById(value:any, id:string) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'therblig',
        'settings': JSON.stringify(value),
        'id': id
      });

      this.callService(this.setRequestService,request);
    }

    public get therbligKeys() {
      return Object.keys(this._plan.therbligs);
    }

    public set therbligKeys(value:any) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'therblig_keys',
        'settings': JSON.stringify(value)
      });

      this.callService(this.setRequestService,request);
    }

    public get therbligs() {
      return this._plan.therbligs;
    }

    public set therbligs(value:any) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'therblig',
        'settings': JSON.stringify(value)
      });

      this.callService(this.setRequestService,request);
    }

    //==========================================================================
    // Macros
    //==========================================================================

    public getMacroById(id:string) {
      return this._plan.macros[id];
    }

    public setMacroById(value:any, id:string) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'macro',
        'settings': JSON.stringify(value),
        'id': id
      });

      this.callService(this.setRequestService,request);
    }

    public get macroKeys() {
      return Object.keys(this._plan.macros);
    }

    public set macroKeys(value:any) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'macro_keys',
        'settings': JSON.stringify(value)
      });

      this.callService(this.setRequestService,request);
    }

    public get macros() {
      return this._plan.macros;
    }

    public set macros(value:any) {
      var request = new ROSLIB.ServiceRequest({
        'type': 'macro',
        'settings': JSON.stringify(value)
      });

      this.callService(this.setRequestService,request);
    }

    //==========================================================================
    //  Derived or One-off Attributes
    //==========================================================================

    public setPoseFromWindow(id:string): Observable<boolean> {
      var request = new ROSLIB.ServiceRequest({
        'type': 'pose',
        'id': id
      });

      return from(this.callService(this.setRequestService,request).then((result) => {
        return true;
      }));
    }

    //==========================================================================
    //  Add Services
    //==========================================================================

    public addTask(name?:string): Observable<string> {
      if (name == undefined) name = "New Task";
      var request = new ROSLIB.ServiceRequest({
        'type': 'task',
        'parameters': JSON.stringify({"name":name})
      });

      return from(this.callService(this.createRequestService,request).then((result) => {
        if (!result.success) {
            this.emitCustomError(result.message);
        }
        return result.id;
      }));
    }

    public addMacro(task): Observable<string> {
      var request = new ROSLIB.ServiceRequest({
        'type': 'macro',
        'parameters': JSON.stringify({"task_id":task})
      });

      return from(this.callService(this.createRequestService,request).then((result) => {
        if (!result.success) {
            this.emitCustomError(result.message);
        }
        return result.id;
      }));
    }

    public addTherblig(task_id:string,index:number,type:string): Observable<string> {
      var request = new ROSLIB.ServiceRequest({
        'type': 'therblig',
        'parameters': JSON.stringify({"task_id":task_id,"index":index,"type":type})
      });
      console.log("Adding therblig "+type+" to task "+task_id+" at index "+index)
      return from(this.callService(this.createRequestService,request).then((result) => {
        if (!result.success) {
            this.emitCustomError(result.message);
        }
        return result.id;
      }));
    }

    public addAgent(): Observable<boolean> {
      var request = new ROSLIB.ServiceRequest({
        'type': 'agent',
        'parameters': '{}'
      });

      return from(this.callService(this.createRequestService,request).then((result) => {
        if (!result.success) {
            this.emitCustomError(result.message);
        }
        return result.id;
      }));
    }

    public addThing(): Observable<boolean> {
      var request = new ROSLIB.ServiceRequest({
        'type': 'thing',
        'parameters': '{}'
      });

      return from(this.callService(this.createRequestService,request).then((result) => {
        if (!result.success) {
            this.emitCustomError(result.message);
        }
        return result.id;
      }));
    }

    public addDestination(): Observable<boolean> {
      var request = new ROSLIB.ServiceRequest({
        'type': 'destination',
        'parameters': '{}'
      });

      return from(this.callService(this.createRequestService,request).then((result) => {
        if (!result.success) {
            this.emitCustomError(result.message);
        }
        return result.id;
      }));
    }

    //==========================================================================
    //  Delete Services
    //==========================================================================

    public deleteTherblig(id:string): Observable<boolean> {
      var request = new ROSLIB.ServiceRequest({
        'type': 'therblig',
        'id': id
      });

      return from(this.callService(this.deleteRequestService,request).then((result) => {
        if (!result.success) {
            this.emitCustomError(result.message);
        }
        return result.success;
      }));
    }

    public deleteTask(id:string): Observable<boolean> {
      var request = new ROSLIB.ServiceRequest({
        'type': 'task',
        'id': id
      });

      return from(this.callService(this.deleteRequestService,request).then((result) => {
        if (!result.success) {
            this.emitCustomError(result.message);
        }
        return result.success;
      }));
    }

    public deleteMacro(id:string): Observable<boolean> {
      var request = new ROSLIB.ServiceRequest({
        'type': 'macro',
        'id': id
      });

      return from(this.callService(this.deleteRequestService,request).then((result) => {
        if (!result.success) {
            this.emitCustomError(result.message);
        }
        return result.success;
      }));
    }

    public deleteThing(id:string): Observable<boolean> {
      var request = new ROSLIB.ServiceRequest({
        'type': 'thing',
        'id': id
      });

      return from(this.callService(this.deleteRequestService,request).then((result) => {
        if (!result.success) {
            this.emitCustomError(result.message);
        }
        return result.success;
      }));
    }

    public deleteDestination(id:string): Observable<boolean> {
      var request = new ROSLIB.ServiceRequest({
        'type': 'destination',
        'id': id
      });

      return from(this.callService(this.deleteRequestService,request).then((result) => {
        if (!result.success) {
            this.emitCustomError(result.message);
        }
        return result.success;
      }));
    }

    public deleteAgent(id:string): Observable<boolean> {
      var request = new ROSLIB.ServiceRequest({
        'type': 'agent',
        'id': id
      });

      return from(this.callService(this.deleteRequestService,request).then((result) => {
        if (!result.success) {
            this.emitCustomError(result.message);
        }
        return result.success;
      }));
    }

    //==========================================================================
    //  Plan
    //==========================================================================

    public get plan(): object {
        return this._plan;
    }

    public forceUpdateRequest(): void {
      console.log("Requesting Data from Backend.");
      let request = new ROSLIB.ServiceRequest({
        'type':"cache",
        'id':''
      })

      this.callService(this.getRequestService,request).then((result) => {
        console.log("Data Received.");
        this.updateFromCache(result.response);
      })
    }

    public get expandedPlan(): Observable<any> {
      var request = new ROSLIB.ServiceRequest({
        'type': 'expanded_plan',
        'id': ''
      });

      return from(this.callService(this.getRequestService,request).then((result) => {
        return JSON.parse(result.response);
      }));
    }

    public uploadPlan(event): void {
        var file = event.target.files[0];
        var reader = new FileReader();

        reader.onload = (e) => {

            let planStr = reader.result.toString();

            let request = new ROSLIB.ServiceRequest({
              'type': 'plan',
              'id': '',
              'settings': planStr
            });

            this.callService(this.setRequestService,request).then((result) => {

              let plan:any;
              try {
                plan = JSON.parse(planStr);
              } catch {
                plan = {};
              }

              let name = plan.name || "Untitled";
              this.snackbar.open(`Loaded '${name}' - ${(result.success) ? "Success" : "Failure"}.`, null, {
                duration: 2000,
              });
            });

        };
        reader.readAsText(file);
    }

    public downloadPlan(event): void {

      let name;
      if (this._plan.name === undefined || this._plan.name === null) {
        name = "Untitled";
      } else {
        name = this._plan.name;
      }

      this.snackbar.open(`Downloading '${name}' as plan.json.`, null, {
        duration: 2000,
      });

      let blob = new Blob([JSON.stringify(this._plan)], {type: "text/plain;charset=utf-8"});
      saveAs(blob, "plan.json");
    }

    //==========================================================================
    //                                 Simulate Action
    //==========================================================================

    public simulate(startTime: number, mode: string,
      feedback_cb: (feedback:any) => void,
      result_cb: (result: any) => void): void
    {
      let goal = new ROSLIB.Goal({
        actionClient: this.simulateClient,
        goalMessage : {
          start_time: startTime,
          mode: mode
        }
      });
      goal.on('feedback',(f) => {feedback_cb(f);});
      goal.on('result', (r) => {
        if (r.status == -1) {
          this.snackbar.open('Simulation: ' + r.msg, null, {duration: 2000,});
        }
        result_cb(r);
      });
      goal.send();
    }

}
