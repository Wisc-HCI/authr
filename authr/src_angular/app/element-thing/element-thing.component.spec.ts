import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { ElementThingComponent } from './element-thing.component';

describe('ElementThingComponent', () => {
  let component: ElementThingComponent;
  let fixture: ComponentFixture<ElementThingComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ ElementThingComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(ElementThingComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
