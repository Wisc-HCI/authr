import { Component, OnInit, Input } from '@angular/core';
import { AuthrService }             from '../services/authr.service';
import { KeyValue }                 from '@angular/common';

@Component({
  selector: 'tof-table',
  templateUrl: './tof-table.component.html',
  styleUrls: ['./tof-table.component.css']
})
export class TofTableComponent {

  constructor(public authrService:AuthrService) { }

  private sortByTimestamp (a: KeyValue<string,any>, b: KeyValue<string,any>): any {
      return a.value.timestamp < b.value.timestamp ? -1 : (b.value.timestamp < a.value.timestamp ? 1 : 0);
  }

  private requestUpdate(source:String,target:String) {
      console.log(source,target);
      this.authrService.refreshTof(source,target);
  }

}
