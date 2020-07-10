import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { ElementDestinationComponent } from './element-destination.component';

describe('ElementDestinationComponent', () => {
  let component: ElementDestinationComponent;
  let fixture: ComponentFixture<ElementDestinationComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ ElementDestinationComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(ElementDestinationComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
