import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { ElementAgentComponent } from './agent.component';

describe('AgentComponent', () => {
  let component: ElementAgentComponent;
  let fixture: ComponentFixture<ElementAgentComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ ElementAgentComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(ElementAgentComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
