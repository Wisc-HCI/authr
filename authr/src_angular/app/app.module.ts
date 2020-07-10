import { BrowserModule }            from '@angular/platform-browser';
import { NgModule }                 from '@angular/core';
import { FormsModule, ReactiveFormsModule }              from '@angular/forms'; // <-- NgModel lives here
import { HttpClientModule}          from '@angular/common/http';
import { MatDialogModule }          from '@angular/material/dialog';

// Modules
import { DragulaModule }            from 'ng2-dragula';
import { BrowserAnimationsModule }  from '@angular/platform-browser/animations';
import { MaterialModule }           from './material.module';
import { MatTooltipModule }         from '@angular/material/tooltip';
import { FlexLayoutModule }         from '@angular/flex-layout';
import { AppRoutingModule }         from './app-routing.module';
import { UtilityModule }            from './utility.module';
import { CoreModule }               from './core/core.module';
import { ColorPickerModule }        from 'ngx-color-picker';
import { AngularSplitModule }       from 'angular-split';
import { AngularResizedEventModule } from 'angular-resize-event';
import { NgbModule }                from '@ng-bootstrap/ng-bootstrap';
import { MatMenuModule } from '@angular/material/menu';

// Components
import { AppComponent }             from './app.component';
import { MainToolbarComponent }     from './main-toolbar/main-toolbar.component';
import { ElementsSidebarComponent } from './elements-sidebar/elements-sidebar.component';
import { TaskDetailComponent }      from './task-detail/task-detail.component';
import { MacroDetailComponent }     from './macro-detail/macro-detail.component';
import { TherbligDetailComponent }  from './therblig-detail/therblig-detail.component';
import { LoginViewComponent }       from './login-view/login-view.component';
import { PlanViewComponent }        from './plan-view/plan-view.component';
import { SimulationViewComponent }  from './simulation-view/simulation-view.component';
import { SetupViewComponent }       from './setup-view/setup-view.component';
import { TaskContentComponent }     from './task-content/task-content.component';
import { TaskComponent }            from './task/task.component';
import { MacroComponent }           from './macro/macro.component';
import { TherbligComponent }        from './therblig/therblig.component';
import { EnvironmentComponent }     from './environment/environment.component';
import { SetupGeneralComponent }    from './setup-general/setup-general.component';
import { SetupAgentsComponent }     from './setup-agents/setup-agents.component';
import { SetupThingsComponent }     from './setup-things/setup-things.component';
import { SetupDestinationsComponent } from './setup-destinations/setup-destinations.component';
import { VectorFieldComponent, VectorInput } from './vector-field/vector-field.component';
import { ElementIconComponent }     from './element-icon/element-icon.component';
import { MatBadgeIconDirective }    from './utility.module';
import { ElementAgentComponent }    from './element-agent/element-agent.component';
import { ElementThingComponent }    from './element-thing/element-thing.component';
import { ElementDestinationComponent } from './element-destination/element-destination.component';
import { TimelineComponent }        from './timeline/timeline.component';
import { TofTableComponent }        from './tof-table/tof-table.component';
import { TherbligPrimitiveDetailComponent } from './therblig-primitive-detail/therblig-primitive-detail.component';
import { TherbligExpandedDetailComponent } from './therblig-expanded-detail/therblig-expanded-detail.component';
import { TrackComponent }     from './track/track.component';

@NgModule({
  declarations: [
    AppComponent,
    MainToolbarComponent,
    ElementsSidebarComponent,
    LoginViewComponent,
    PlanViewComponent,
    SimulationViewComponent,
    SetupViewComponent,
    TaskContentComponent,
    TaskComponent,
    TaskDetailComponent,
    MacroComponent,
    MacroDetailComponent,
    TherbligDetailComponent,
    TherbligComponent,
    EnvironmentComponent,
    SetupGeneralComponent,
    SetupAgentsComponent,
    SetupThingsComponent,
    SetupDestinationsComponent,
    VectorFieldComponent,
    VectorInput,
    ElementIconComponent,
    MatBadgeIconDirective,
    ElementAgentComponent,
    ElementThingComponent,
    ElementDestinationComponent,
    TimelineComponent,
    TofTableComponent,
    TherbligPrimitiveDetailComponent,
    TherbligExpandedDetailComponent,
    TrackComponent
  ],
  imports: [
    BrowserModule.withServerTransition({appId: 'authr'}),
    FormsModule,
    AppRoutingModule,
    BrowserAnimationsModule,
    MaterialModule,
    MatTooltipModule,
    FlexLayoutModule,
    UtilityModule,
    MatDialogModule,
    HttpClientModule,
    ReactiveFormsModule,
    ColorPickerModule,
    AngularResizedEventModule,
    AngularSplitModule.forRoot(),
    CoreModule.forRoot({}),
    DragulaModule.forRoot(),
    NgbModule,
    MatMenuModule,
  ],
  entryComponents: [
    VectorFieldComponent,
    VectorInput,
  ],
  providers: [ ],
  bootstrap: [AppComponent]
})
export class AppModule { }
