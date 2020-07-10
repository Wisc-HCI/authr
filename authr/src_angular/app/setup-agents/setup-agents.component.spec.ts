import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { SetupAgentsComponent } from './setup-agents.component';

describe('SetupAgentsComponent', () => {
  let component: SetupAgentsComponent;
  let fixture: ComponentFixture<SetupAgentsComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ SetupAgentsComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(SetupAgentsComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
