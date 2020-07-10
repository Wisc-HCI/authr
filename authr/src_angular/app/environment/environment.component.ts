import { Component, OnInit, HostListener, OnChanges, OnDestroy, SimpleChanges, Input } from '@angular/core';
import { Observable, from, of } from 'rxjs';
import { AuthrService } from '../services/authr.service';
import * as ROS3D from 'ros3d';
import * as ROSLIB from 'roslib';

@Component({
  selector: 'environment',
  templateUrl: './environment.component.html',
  styleUrls: ['./environment.component.css'],
  host: {'class': 'content-flex'}
})
export class EnvironmentComponent implements OnInit, OnDestroy, OnChanges {

  public viewer: ROS3D.Viewer;
  @Input() enabled: boolean = false;

  constructor(public authrService: AuthrService) { }

  ngOnInit() {
  }

  ngOnChanges(changes: SimpleChanges) {
     for (let propName in changes) {
	        let change = changes[propName];
            if (propName = 'enabled') {
                this.initializeEnvironment()
            }
     }
  }

  ngOnDestroy() {
    this.authrService.teardownClients();
  }

  public initializeEnvironment() {

      if (this.authrService.connected) {

        // Initialize the viewer
        this.viewer = new ROS3D.Viewer({
            divID: 'environment',
            width: 600,
            height: 300,
            antialias: true,
            background: '#303030',
            intensity: .65
        });

        this.viewer.addObject(new ROS3D.Grid({
            color: '#0181c4',
            cellSize: 0.5,
            num_cells: 20,
            lineWidth: 2
        }));

        this.adjustSize()

        this.authrService.setupClients(this.viewer,true).subscribe(r => {
          setTimeout(() => {
            this.authrService.environmentRefreshTopic.publish({data:'marker'});
            this.authrService.interactiveMarkerRefreshTopic.publish({data:'all'});
            this.authrService.simpleMarkerRefreshTopic.publish({data:'all'});
            this.authrService.visualizationMarkerRefreshTopic.publish({data:'all'});
          },450);
        });
      }
  }

  @HostListener('window:resize', ['$event'])
  adjustSize(event?) {
      if (this.viewer) {
          let element = document.getElementById('environment');
          this.viewer.resize(element.offsetWidth,element.offsetHeight);
      }
  }

}
